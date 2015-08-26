#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
import cv2
import Queue
import os
import inspect
import sys
import numpy as np
import multiprocessing
#Add script directory to path
script_dir =  os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) # script directory
sys.path.append(script_dir)

#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_video import VN_video
from Common.VN_dispatcher import VN_dispatcher
from Common.VN_logger import VN_logger
from Common.VN_util import *
from Common.VN_position_vector import PositionVector
from Common.VN_vehicle_control import veh_control

#PRECISION LAND IMPORTS
from PrecisionLand_lib.PL_gui import PrecisionLandGUI as gui
from PrecisionLand_lib.PL_sim import sim
from PrecisionLand_lib.Ring_Detector import Ring_Detector, Point, Circle, Ring

#DRONEAPI IMPORTS
from droneapi.lib import VehicleMode, Location, Attitude



'''
TODO:
-Add timestamp/sync
-update config file

Future:
-add parameter set over mavlink

Improvements:
-fix config file naming/loading(make it more intelligent)
'''


class PrecisionLand(object):

    def __init__(self):

        #load config file
        VN_config.get_file('Smart_Camera')

        #get camera specs
        self.camera_index = VN_config.get_integer('camera','index',0)
        self.camera_width = VN_config.get_integer('camera', 'width', 640)
        self.camera_height = VN_config.get_integer('camera', 'height', 480)
        self.camera_hfov = VN_config.get_float('camera', 'horizontal-fov', 72)
        self.camera_vfov = VN_config.get_float('camera', 'vertical-fov', 43)

        #simulator
        self.simulator = VN_config.get_boolean('simulator','use_simulator',True)
        self.target_file = VN_config.get_string('simulator', 'target_location', os.environ['HOME'] + '/visnav/target.jpg')
        self.target_size = VN_config.get_float('algorithm', 'outer_ring', 1.0)

        #The number of core to be used while processing image data
		#This number may be less than the actaul number of cores on the CPU depending on the users specifications
		#cores = min(desiredCores, multiprocessing.cpu_count()) //dont allow more cores than the CPU has available and don;t run more than the user wants
        desired_cores = VN_config.get_integer('processing', 'desired_cores', 4)
        available_cores = min(desired_cores, multiprocessing.cpu_count())
		#check if a core is already in use for background image capture
        cores_image_capture = int(VN_config.get_boolean('processing','background_capture', True))
        self.cores_processing = max(available_cores - cores_image_capture,1)

        #Run the program no matter what mode or location; Useful for debug purposes
        self.always_run = VN_config.get_boolean('general', 'always_run', True)

        #how many frames have been captured
        self.frames_captured = 0


    def name(self):
        return "Precision_Land"

    def connect(self):
        while(veh_control.is_connected() == False):
            # connect to droneapi
            veh_control.connect(local_connect())
        self.vehicle = veh_control.get_vehicle()

    def run(self):

        VN_logger.text(VN_logger.GENERAL, 'Running {0}'.format(self.name()))

        #start a video capture
        if(self.simulator):
            VN_logger.text(VN_logger.GENERAL, 'Using simulator')
            sim.load_target(self.target_file, self.target_size)
            sim.set_target_location(veh_control.get_home(True))
            #sim.set_target_location(Location(0,0,0))

        else:
            VN_video.start_capture(self.camera_index)


        #create an image processor
        #detector = CircleDetector()
        detector = Ring_Detector()

        #create a queue for images
        imageQueue = []

        #initilize autopilot variables
        location = Location(0,0,0)
        attitude = Attitude(0,0,0)


        while veh_control.is_connected():

            #we are in a landing mode and we are still running the landing program
            if (self.always_run) or (veh_control.get_mode() == "LAND") or (veh_control.get_mode() == "RTL"):

                #update how often we dispatch a command
                VN_dispatcher.calculate_dispatch_schedule()

                #update simulator
                if(self.simulator):
                    #get info from autopilot
                    location = veh_control.get_location()
                    attitude = veh_control.get_attitude()
                    sim.refresh_simulator(location,attitude)

                # grab an image
                frame = None
                capStart = current_milli_time()
                if(self.simulator):
                    frame = sim.get_frame()
                else:
                    frame = VN_video.get_image()
                capStop = current_milli_time()

                #multicore enhancements
                if self.cores_processing > 1:
                    #update capture time
                    VN_dispatcher.update_capture_time(capStop-capStart)


                    #Process image
                    #We schedule the process as opposed to waiting for an available core
                    #This brings consistancy and prevents overwriting a dead process before
                    #information has been grabbed from the Pipe
                    if VN_dispatcher.is_ready():
                        #queue the image for later use: displaying image, overlays, recording
                        imageQueue.append((frame,self.frames_captured))

                        #the function must be run directly from the class
                        VN_dispatcher.dispatch(target=detector.analyze_frame_async, args=(frame,self.frames_captured))

                        self.frames_captured += 1


                    #retreive results
                    if VN_dispatcher.is_available():

                        #results of image processor
                        results = VN_dispatcher.retreive()
                        img, frame_id = None, None
                        for f in imageQueue:
                            img, frame_id = f
                            if results[0] == frame_id:
                                imageQueue.remove(f)
                                break

                        self.process_results(results, img, frame_id)

                #single core
                else:
                    results = detector.analyze_frame(frame, self.frames_captured)
                    self.process_results(results, frame, self.frames_captured)
                    self.frames_captured += 1

            else:
                VN_logger.text(VN_logger.GENERAL, 'Not in landing mode')



        #terminate program
        VN_logger.text(VN_logger.GENERAL, 'Vehicle disconnected, Program Terminated')
        if(self.simulator == False):
            VN_video.stop_capture()

    def process_results(self,results, img, frame_id):

        VN_logger.text(VN_logger.GENERAL, 'Frame {0}'.format(frame_id))

        best_ring = results[2]
        rings = results[3]

        #overlay gui
        rend_Image = gui.add_ring_highlights(img, rings,best_ring)
        yaw , radius = None, None
        if best_ring is not None:
            radius = best_ring.radius
            if best_ring.is_valid():
                yaw = int(math.degrees(best_ring.orientation))
        status_text = '{0} Rings\n{1} ms\n{2} degs\n{3} radius\n{4} meters'.format(len(results[3]), results[1], yaw, radius, int(veh_control.get_location().alt))
        rend_Image = gui.add_stats(rend_Image,status_text, 5, 250)

        #show/record images
        VN_logger.image(VN_logger.RAW, img)
        VN_logger.image(VN_logger.GUI, rend_Image)

        #display/log data
        VN_logger.text(VN_logger.ALGORITHM, status_text.replace('\n', ', '))

        #send results if we found the landing pad
        if(best_ring is not None):
            x_angle, y_angle, yaw = best_ring.get_angular_offset(self.camera_width,self.camera_height,self.camera_hfov,self.camera_vfov)

            #send commands to autopilot
            veh_control.report_landing_target(x_angle, y_angle,0,0,0)


# if starting from mavproxy
if __name__ == "__builtin__":
    # start precision landing
    strat = PrecisionLand()

    # connect to droneapi
    VN_logger.text(VN_logger.GENERAL, 'Connecting to vehicle...')
    strat.connect()
    VN_logger.text(VN_logger.GENERAL, 'Vehicle connected!')

    # run strategy
    strat.run()
