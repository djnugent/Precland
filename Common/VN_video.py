#!/usr/bin/python
#SYSTEM IMPORTS
import sys
from os.path import expanduser
import time
import math
import multiprocessing
import cv2
import numpy as np

#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_logger import VN_logger


"""
VN_video.py
This file includes functions to:
    Initialise the camera
    Initialise the output video
Image size is held in the smart_camera.cnf
"""


class VizNavVideo:

    def __init__(self):

        # get which camera we will use
        self.camera_index = VN_config.get_integer('camera','index',0)

        # get image resolution
        self.img_width = VN_config.get_integer('camera','width',640)
        self.img_height = VN_config.get_integer('camera','height',480)
        

        # get image center
        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2
        
        
        # define field of view
        self.cam_hfov = VN_config.get_float('camera','horizontal-fov',70.42)
        self.cam_vfov = VN_config.get_float('camera','vertical-fov',43.3)
        


        #get camera distortion matrix and intrinsics. Defaults: logitech c920
        mtx = np.array([[ 614.01269552,0,315.00073982],
                [0,614.43556296,237.14926858],
                [0,0,1.0]])
        dist = np.array([0.12269303, -0.26618881,0.00129035, 0.00081791,0.17005303])

        self.matrix  = VN_config.get_array('camera','matrix', mtx)
        self.distortion = VN_config.get_array('camera', 'distortion', dist)

        self.newcameramtx, self.roi=cv2.getOptimalNewCameraMatrix(self.matrix,self.distortion,(self.img_width,self.img_height),1,(self.img_width,self.img_height))


        #create a camera object
        self.camera = None


        #number of cores available for use
        desiredCores = VN_config.get_integer('processing', 'desired_cores', 4)
        self.cores_available = min(desiredCores, multiprocessing.cpu_count())


        #does the user want to capture images in the background
        self.background_capture = VN_config.get_boolean('processing','background_capture', True)


        # background image processing variables
        self.proc = None              # background process object
        self.parent_conn = None       # parent end of communicatoin pipe
        self.img_counter = 0          # num images requested so far
        self.is_backgroundCap = False #state variable for background capture
 

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraVideo Object W:%d H:%d" % (self.img_width, self.img_height)

    # get_camera - initialises camera and returns VideoCapture object 
    def get_camera(self,index):
        VN_logger.text(VN_logger.GENERAL, 'Starting Camera....')


        # setup video capture
        self.camera = cv2.VideoCapture(index)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)

        # check we can connect to camera
        if not self.camera.isOpened():
            VN_logger.text(VN_logger.GENERAL,"failed to open camera, exiting!")
            sys.exit(0)

        VN_logger.text(VN_logger.GENERAL, 'Camera Open!')

        return self.camera


    #
    # background image processing routines
    #

    # image_capture_background - captures all images from the camera in the background and returning the latest image via the pipe when the parent requests it
    def image_capture_background(self, imgcap_connection):
        # exit immediately if imgcap_connection is invalid
        if imgcap_connection is None:
            VN_logger.text(VN_logger.GENERAL, "image_capture failed because pipe is uninitialised")
            return

        # clear latest image
        latest_image = None

        while True:
            # constantly get the image from the webcam
            success_flag, image=self.camera.read()

            # if successful overwrite our latest image
            if success_flag:
                latest_image = image

            # check if the parent wants the image
            if imgcap_connection.poll():
                recv_obj = imgcap_connection.recv()
                # if -1 is received we exit
                if recv_obj == -1:
                    break

                # otherwise we return the latest image
                imgcap_connection.send(latest_image)

        # release camera when exiting
        self.camera.release()


    def stop_capture(self):
        #Clean up when exitting background capture
        if(self.is_backgroundCap):
            # send exit command to image capture process
            self.parent_conn.send(-1)

            # join process
            self.proc.join()
        #no clean up required with regular capture

    def start_capture(self,index = 0):
        #make sure a camera is intialized
        if self.camera is None:
            self.get_camera(index)

        #background capture is desired
        if self.background_capture:
            #if we have more than one core available, then start background capture
            if(self.cores_available > 1):

                # create pipe
                self.parent_conn, imgcap_conn = multiprocessing.Pipe()

                # create and start the sub process and pass it it's end of the pipe
                self.proc = multiprocessing.Process(target=self.image_capture_background, args=(imgcap_conn,))
                self.proc.daemon = True
                self.proc.start()

                #Mark that we are in background capture mode
                self.is_backgroundCap = True
        else:
            #Not enough cores for background capture or just doing regular capture
            self.is_backgroundCap = False


    # get_image - returns latest image from the camera captured from the background process
    def get_image(self):
        #grab image from pipe of background capture
        if(self.is_backgroundCap):
            # return immediately if pipe is not initialised
            if self.parent_conn == None:
                return None

            # send request to image capture for image
            self.parent_conn.send(self.img_counter)

            # increment counter for next interation
            self.img_counter = self.img_counter + 1

            # wait endlessly until image is returned
            img = self.parent_conn.recv()

        #use standard image cap
        else:
            #Grab an image
            success_flag, img= self.camera.read()

        # return image to caller
        return img

    #undisort_image- removes any distortion caused by the camera lense
    def undisort_image(self,frame):
        #undistort
        dst = cv2.undistort(frame, self.matrix, self.distortion, None, self.newcameramtx)

        # crop the image
        x,y,w,h = self.roi
        dst = dst[y:y+h, x:x+w]

        return dst

    # main - tests SmartCameraVideo class
    def main(self):
        #open a camera
        #self.get_camera(0)

        # start background process
        self.start_capture(self.camera_index)

        #did we start background capture
        print 'Background capture {0}'.format(self.is_backgroundCap)

        while True:
            # send request to image capture for image
            img = self.get_image()

            #undistort image
            img = self.undisort_image(img)
    
            # check image is valid
            if not img is None:
                # display image
                cv2.imshow ('image_display', img)
            else:
                print "no image"
    
            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
    
            # take a rest for a bit
            time.sleep(0.1)

        # send exit command to image capture process
        self.stop_capture()

        print "a2p 10 = %f" % self.angle_to_pixels_x(10)
        print "p2a 10 = %f" % self.pixels_to_angle_x(10)

# create a single global object
VN_video = VizNavVideo()

if __name__ == "__main__":
    VN_video.main()