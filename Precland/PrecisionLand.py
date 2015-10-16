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

#CV IMPORTS
from cv_utils.config import Config
from cv_utils.video import Video
from cv_utils.logger import Logger
from cv_utils.vehicle_control import VehicleControl
from cv_utils.dispatcher import Dispatcher
from cv_utils.helpers import *
from cv_utils.dataTypes import *

#LOCAL IMPORTS
from Simulator import PrecisionLandSimulator
from GUI import PrecisionLandGUI as gui
from Land_Control import Land_Control
from Ring_Detector import *


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

    def __init__(self,config):


        #Unpack Config file
        self.config = config
        #camera specs
        self.camera_src = self.config.get_string('camera','source',"0")
        self.camera_width = self.config.get_integer('camera', 'width', 640)
        self.camera_height = self.config.get_integer('camera', 'height', 480)
        self.camera_hfov = self.config.get_float('camera', 'horizontal-fov', 72)
        self.camera_vfov = self.config.get_float('camera', 'vertical-fov', 43)
        self.camera_frame_rate = self.config.get_integer('camera','framerate', 30)

        #simulator
        self.use_simulator = self.config.get_boolean('simulator','use_simulator',True)
        self.target_file = self.config.get_string('simulator', 'target_location', '~/visnav/target.jpg')
        self.target_size = self.config.get_float('simulator', 'target_size', 1.0)
        self.sim = PrecisionLandSimulator(self.camera_width,self.camera_height,self.camera_hfov,self.camera_vfov,30)

        #processing
        desired_cores = self.config.get_integer('processing', 'desired_cores', 4)
        background_capture = self.config.get_boolean('processing','background_capture', True)
    	self.dispatcher = Dispatcher(desired_cores)

        #general
        #Run the program no matter what mode or location; Useful for debug purposes
        self.always_run = self.config.get_boolean('general', 'always_run', True)
        #send target's body frame angles or absolute lat/lon
        self.operation_mode = self.config.get_string('general', 'operation_mode', 'velocity')

        #logging
        #create a logger
        self.logger = Logger("precland",self.config.get_string('logging', 'location', '~/precland/'))
        self.logger.set_print_level(self.config.get_string('logging', 'print_level', 'general, debug'))
    	self.logger.set_log_level(self.config.get_string('logging', 'log_level', 'general, debug, aircraft'))
    	self.logger.set_display_level(self.config.get_string('logging', 'display_level', 'gui, raw'))
    	self.logger.set_record_level(self.config.get_string('logging', 'record_level', 'gui, raw'))
    	self.logger.set_record_type(self.config.get_string('logging', 'record_type', 'video'))

        #vehicle
        self.veh_control = VehicleControl()
        self.land_control = Land_Control(self.veh_control)

        #video
        self.video = Video(self.camera_src,(self.dispatcher.remaining_cores > 0 and background_capture))

        #how many frames have been captured
        self.frames_captured = 0

    def name(self):
        return "Precision_Land"

    def connect(self):
        self.logger.text(self.logger.GENERAL, 'Connecting to vehicle...')
        while(self.veh_control.is_connected() == False):
            # connect to droneapi
            self.veh_control.connect(local_connect())
        self.logger.text(self.logger.GENERAL, 'Vehicle connected!')


    def run(self):

        self.logger.text(self.logger.GENERAL, 'Running {0}'.format(self.name()))

        #start a video capture
        if(self.use_simulator):
            self.logger.text(self.logger.GENERAL, 'Using simulator')
            self.sim.load_target(self.target_file, self.target_size)
            self.sim.set_target_location(self.veh_control.get_home(True))
            #self.sim.set_target_location(Location(0,0,0))

        else:
            self.video.start_capture()


        #create an image processor
        #detector = CircleDetector()
        detector = Ring_Detector(self.config)

        #create a queue for images
        imageQueue = []

        #initilize autopilot variables
        location = Location(0,0,0)
        attitude = Attitude(0,0,0)


        while self.veh_control.is_connected():

            #we are in a landing mode and we are still running the landing program
            if (self.always_run or self.veh_control.get_mode() == "LAND" or self.veh_control.get_mode() == "RTL") and self.veh_control.is_armed():

                #update how often we dispatch a command
                self.dispatcher.calculate_dispatch_schedule()

                #update simulator
                if(self.use_simulator):
                    #get info from autopilot
                    location = self.veh_control.get_location()
                    attitude = self.veh_control.get_attitude()
                    self.sim.refresh_simulator(location,attitude)

                # grab an image
                frame = None
                capStart = time.time() * 1000
                if(self.use_simulator):
                    frame = self.sim.get_frame()
                else:
                    frame = video.get_image()
                capStop = time.time() * 1000

                #grab additional info
                altitude = 0
                timestamp = 0
                if self.use_simulator:
                    altitude = location.alt
                    timestamp = 0

                elif self.camera_src == "PX4flow": #flow
                    cam = video.get_camera()
                    timestamp = cam.get_timestamp()
                    '''
                    altitude = cam.get_lidar()
                    #compensated for angle of range finder altitude
                    attitude = self.veh_control.get_attitude(timestamp)
                    temp = math.cos(attitude.pitch) * math.cos(attitude.roll)
                    temp = max(temp, 0.707)
                    rng_alt = rng_alt * temp
                    '''
#FIXME
                    altitude = self.veh_control.get_location().alt
                    if altitude is None:
                        altitude = 0

                else: #webcam
                    altitude = 0
                    timestamp = 0

                #multicore enhancements
                if self.cores_processing > 1:
                    #update capture time
                    self.dispatcher.update_capture_time(capStop-capStart)


                    #Process image
                    #We schedule the process as opposed to waiting for an available core
                    #This brings consistancy and prevents overwriting a dead process before
                    #information has been grabbed from the Pipe
                    if self.dispatcher.is_ready():
                        #queue the image for later use: displaying image, overlays, recording
                        imageQueue.append((frame,self.frames_captured))

                        #the function must be run directly from the class
                        self.dispatcher.dispatch(target=detector.analyze_frame_async, args=(frame,self.frames_captured,timestamp,altitude))

                        self.frames_captured += 1


                    #retreive results
                    if self.dispatcher.is_available():

                        #results of image processor
                        results = self.dispatcher.retreive()
                        img, frame_id = None, None
                        for f in imageQueue:
                            img, frame_id = f
                            if results[0] == frame_id:
                                imageQueue.remove(f)
                                break

                        self.process_results(results, img)

                #single core
                else:
                    results = detector.analyze_frame(frame,self.frames_captured,timestamp,altitude)
                    self.process_results(results, frame)
                    self.frames_captured += 1

            else:
                self.logger.text(self.logger.GENERAL, 'Not in landing mode')
                time.sleep(0.5)


        #terminate program
        self.logger.text(self.logger.GENERAL, 'Vehicle disconnected, Program Terminated')
        if(self.use_simulator == False):
            video.stop_capture()



    # process_results - Act on information extracted from image
    def process_results(self,results, img):
        #unpack data
        frame_id, timestamp, altitude = results[0]
        best_ring = results[2]
        rings = results[3]


        #overlay gui
        rend_Image = gui.add_ring_highlights(img, rings,best_ring)
        yaw , radius = None, None
        if best_ring is not None:
            radius = best_ring.radius
            if best_ring.is_valid():
                yaw = int(math.degrees(best_ring.orientation))
        status_text = '{0} Rings\n{1} ms\n{2} degs\n{3} radius\n{4} meters'.format(len(results[3]), results[1], yaw, radius, int(altitude))
        rend_Image = gui.add_stats(rend_Image,status_text, 5, 250)

        #show/record images
        self.logger.image(self.logger.RAW, img)
        self.logger.image(self.logger.GUI, rend_Image)

        #display/log data
        self.logger.text(self.logger.GENERAL, 'Frame {0}'.format(frame_id))
        self.logger.text(self.logger.ALGORITHM, status_text.replace('\n', ', '))

        #send results if we found the landing pad
        if(best_ring is not None):

            bf_angle_offset = best_ring.get_angular_offset(self.camera_width,self.camera_height,self.camera_hfov,self.camera_vfov)
            coord_frame = None
            #send raw target angular offsets to autopilot
            if self.operation_mode == 'angular':
                x_targ, y_targ = bf_angle_offset.x, bf_angle_offset.y
                coord_frame = mavlink.MAV_FRAME_BODY_NED
                #send commands to autopilot
                self.veh_control.report_landing_target(timestamp,coord_frame,x_targ, y_targ,altitude,0,0)

            #send absolute target location to autopilot
            elif self.operation_mode == 'global':

                #interpolate location and attitude
                veh_loc = self.veh_control.get_location()#timestamp)
                veh_home = self.veh_control.get_home()
                veh_att = self.veh_control.get_attitude()#timestamp)

                loc = camera_to_relative(bf_angle_offset,veh_loc,veh_att,altitude,veh_home)
                x_targ, y_targ = loc.x, loc.y
                coord_frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                #send commands to autopilot
                self.veh_control.report_landing_target(timestamp,coord_frame,x_targ, y_targ,altitude,0,0)

            elif self.operation_mode == 'velocity':
                self.land_control.consume_target_offset(bf_angle_offset,timestamp)
        else:
            if self.operation_mode == 'velocity':
                self.land_control.update()




# if starting from mavproxy
if __name__ == "__builtin__":
    #load config file
    config = Config("precland","~/precland.cnf")

    # start precision landing
    strat = PrecisionLand(config)

    # connect to droneapi
    strat.connect()

    # run strategy
    strat.run()
