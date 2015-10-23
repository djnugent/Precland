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
from pymavlink.mavutil import mavlink

#Add script directory to path
script_dir =  os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) # script directory
sys.path.append(script_dir)

#CV IMPORTS
from cv_utils.config import Config
from cv_utils.video import Video
from cv_utils.logger import Logger
from cv_utils.vehicle_control import VehicleControl
from cv_utils.transforms import *
from cv_utils.dataTypes import *
from cv_utils.threader import *

#LOCAL IMPORTS
from Simulator import PrecisionLandSimulator
from Land_Control import Land_Control
from Ring_Detector import *


#DRONEAPI IMPORTS
from droneapi.lib import VehicleMode, Location, Attitude


'''
TODO:
-re-add timestamp/sync
-update config file

Future:
-add parameter set over mavlink
'''


class PrecisionLand(object):

    def __init__(self,config):

        #Unpack Config file
        self.config = config

        #simulator
        self.use_simulator = self.config.get_boolean('simulator','use_simulator',True)

        #general
        #Run the program no matter what mode or location; Useful for debug purposes
        self.always_run = self.config.get_boolean('general', 'always_run', True)

        #logging
        #create a logger and set log levels
        self.logger = Logger("precland",self.config.get_string('logging', 'location', '~/precland/'))
        self.logger.set_print_level(self.config.get_string('logging', 'print_level', 'general, debug'))
    	self.logger.set_log_level(self.config.get_string('logging', 'log_level', 'general, debug, aircraft'))
    	self.logger.set_display_level(self.config.get_string('logging', 'display_level', 'gui, raw'))
    	self.logger.set_record_level(self.config.get_string('logging', 'record_level', 'gui, raw'))
    	self.logger.set_record_type(self.config.get_string('logging', 'record_type', 'video'))

        #video
        self.camera_src = self.config.get_string('camera','source', "0")
        self.background_capture = self.config.get_boolean('camera','background_capture', False)
        self.hfov = config.get_float('camera','hfov',72.3)
        self.vfov = config.get_float('camera','vfov',46)
        self.video = Video(self.camera_src,self.background_capture)

        #non opencv capture device
        if self.camera_src == 'PX4Flow':
            from Flow_Camera import flow_cam
            self.video.set_camera(flow_cam)


    def name(self):
        return "Precision Land"

    def connect(self):
        self.logger.text(self.logger.GENERAL, 'Connecting to vehicle...')
        self.veh_control = VehicleControl()
        while(self.veh_control.is_connected() == False):
            # connect to droneapi
            self.veh_control.connect(local_connect())
        self.logger.text(self.logger.GENERAL, 'Vehicle connected!')

    def run(self):
        #start a video capture
        self.frames_captured = 0 #how many frames have been captured
        if(self.use_simulator):
            self.logger.text(self.logger.GENERAL, 'Using simulator')
            self.sim = PrecisionLandSimulator(self.config)
            self.sim.set_target_location(self.veh_control.get_home(True))
        else:
            self.video.start_capture()


        #create an image processor
        detector = Ring_Detector(self.config)

        #create a queue for images
        imageQueue = []

        #initilize autopilot variables
        location = Location(0,0,0)
        attitude = Attitude(0,0,0)

        #control the vehicle state in the background
        self.land_control = Land_Control(self.config, self.veh_control)
        control_thread = Threader(target=self.land_control.update, args=None,iterations = -1)

        while self.veh_control.is_connected():

            #start thread
            if not control_thread.is_alive():
                control_thread.start()

            #we are in a landing mode and we are still running the landing program
            if (self.always_run or self.veh_control.get_mode() == "LAND" or self.veh_control.get_mode() == "RTL") and self.veh_control.is_armed():

                #grab vehicle pos
                ret, location = self.veh_control.get_location()
                ret, attitude = self.veh_control.get_attitude()
                altitude = location.alt #alt above terrain not used...yet
                timestamp = 0 # not used...yet

                # grab an image
                frame = None
                if(self.use_simulator):
                    self.sim.set_target_location(self.veh_control.get_home(True))
                    self.sim.refresh_simulator(location,attitude)
                    ret,frame = self.sim.get_image()
                else:
                    ret,frame = self.video.get_image()

                #run target detector
                results = detector.analyze_frame(frame,self.frames_captured,timestamp,altitude)

                #process image data
                self.process_results(results, frame)
                self.frames_captured += 1

            else:
                self.logger.text(self.logger.GENERAL, 'Not in landing mode')
                time.sleep(0.5)

        #terminate program
        self.logger.text(self.logger.GENERAL, 'Vehicle disconnected, Program Terminated')
        Threader.stop_all()
        if(self.use_simulator == False):
            self.video.stop_capture()



    # process_results - Act on information extracted from image
    def process_results(self,results, img):
        #unpack data
        frame_id, timestamp, altitude = results[0]
        best_ring = results[2]
        rings = results[3]
        img_height = img.shape[0]
        img_width = img.shape[1]


        yaw , radius = None, None
        #create a shallow copy of img
        rend_Image = np.copy(img)
        if best_ring is not None:
            for r in rings:
                rend_Image = r.render_overlay(rend_Image,(255,0,0))
            rend_Image = best_ring.render_overlay(rend_Image,(0,0,255))
            radius = best_ring.radius
            if best_ring.is_valid():
                yaw = int(math.degrees(results[2].orientation))

        #overlay stats
        status_text = '{0} Rings\n{1} ms\n{2} degs\n{3} radius\n{4} meters'.format(len(results[3]), results[1], yaw, radius, int(altitude))
        rend_Image = add_stats(rend_Image,status_text, 5, 250)

        #show/record images
        self.logger.image(self.logger.RAW, img)
        self.logger.image(self.logger.GUI, rend_Image)

        #display/log data
        self.logger.text(self.logger.GENERAL, 'Frame {0}'.format(frame_id))
        self.logger.text(self.logger.ALGORITHM, status_text.replace('\n', ', '))

        #feed our land controller with new info if we have it
        if best_ring is not None:
            angular_offset = best_ring.get_angular_offset(img_width, img_height, self.hfov, self.vfov)
            self.land_control.consume_target_offset(angular_offset,timestamp)



# if starting from mavproxy
if __name__ == "__builtin__":
    #load config file
    config = Config("precland","~/precland/precland_default.cnf")

    # start precision landing
    strat = PrecisionLand(config)

    # connect to droneapi
    strat.connect()

    # run strategy
    strat.run()
