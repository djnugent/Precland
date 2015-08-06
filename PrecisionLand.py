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
from copy import copy

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
from PrecisionLand_lib.CircleDetector import CircleDetector

#DRONEAPI IMPORTS
from droneapi.lib import VehicleMode, Location, Attitude


'''
Temporary Changes:
-added kill_camera(commented out)
'''


'''
TODO:
-update config file

Future:
-add parameter set over mavlink


Improvements:
-fix config file naming/loading(make it more intelligent)
-add better target_detected logic(multiple frames required for a lock)
-add update rate to VN_logger
-fix Logging printing to console
-handle droneapi start up better(location being null at start up-> issue using see inside_landing_area() RIGHT at startup)
'''


class PrecisionLand(object):

	def __init__(self):

		#load config file
		VN_config.get_file('Smart_Camera')

		#get camera specs
		self.camera_index = VN_config.get_integer('camera','camera_index',0)
		self.camera_width = VN_config.get_integer('camera', 'camera_width', 640)
		self.camera_height = VN_config.get_integer('camera', 'camera_height', 480)
		self.camera_hfov = VN_config.get_float('camera', 'horizontal-fov', 72.42)
		self.camera_vfov = VN_config.get_float('camera', 'vertical-fov', 43.3)

		#simulator
		self.simulator = VN_config.get_boolean('simulator','use_simulator',True)
		self.target_file = VN_config.get_string('simulator', 'target_location', os.environ['HOME'] + '/visnav/target.jpg')
		self.target_size = VN_config.get_float('algorithm', 'outer_ring', 1.0)

		#Run the program no matter what mode or location; Useful for debug purposes
		self.always_run = VN_config.get_boolean('general', 'always_run', True)

		#how many frames have been captured
		self.frame_count = 0

		#debugging:
		self.kill_camera = False

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
		detector = CircleDetector()

		#create a queue for images
		imageQueue = Queue.Queue()

		#initilize autopilot variables
		location = Location(0,0,0)
		attitude = Attitude(0,0,0)


	 	while veh_control.is_connected():

	 		'''
	 		#kill camera for testing
	 		if(cv2.waitKey(2) == 1113938):
				self.kill_camera =  not self.kill_camera
			'''

	 		#we are in the landing zone or in a landing mode and we are still running the landing program
	 		#just because the program is running does not mean it controls the vehicle
	 		#i.e. in the landing area but not in a landing mode
	 		#FIXME add inside_landing_area() back to conditional
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

				'''
				ret,thresh = cv2.threshold(frame,32,255,cv2.THRESH_BINARY)
				cv2.imshow("thresh", thresh)
				cv2.waitKey(1)
				kernel = np.ones((5,5),np.uint8)
				erode = cv2.dilate(frame,kernel)
				cv2.imshow("erode", erode)
				cv2.waitKey(1)


				#debug
				edges = cv2.Canny(erode,100,200,3)

				contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
				img = copy(frame)
				cv2.drawContours(img,contours,-1,(0,255,0),3)
				cv2.imshow("cont", img)
				cv2.waitKey(1)
				'''
				'''
				if(self.kill_camera):
					frame[:] = (0,255,0)
				'''

		 		#update capture time
		 		VN_dispatcher.update_capture_time(capStop-capStart)


				#Process image
				#We schedule the process as opposed to waiting for an available core
				#This brings consistancy and prevents overwriting a dead process before
				#information has been grabbed from the Pipe
				if VN_dispatcher.is_ready():
					#queue the image for later use: displaying image, overlays, recording
					imageQueue.put(frame)

					#the function must be run directly from the class
					VN_dispatcher.dispatch(target=detector.analyze_frame, args=(frame,attitude,))



		 		#retreive results
		 		if VN_dispatcher.is_available():

		 			VN_logger.text(VN_logger.GENERAL, 'Frame {0}'.format(self.frame_count))
		 			self.frame_count += 1


		 			#results of image processor
		 			results = VN_dispatcher.retreive()
		 			# get image that was passed with the image processor
		 			img = imageQueue.get()

		 			#overlay gui
		 			rend_Image = gui.add_target_highlights(img, results[3])


		 			#show/record images
		 			VN_logger.image(VN_logger.RAW, img)
		 			VN_logger.image(VN_logger.GUI, rend_Image)

		 			#display/log data
		 			VN_logger.text(VN_logger.ALGORITHM,'RunTime: {0} Center: {1} Distance: {2} Raw Target: {3}'.format(results[0],results[1],results[2],results[3]))

		 			#send results if we found the landing pad
		 			if(results[1] is not None):
		 				#shift origin to center of the image
		 				x_pixel = results[1][0] - (self.camera_width/2.0)
		 				y_pixel = results[1][1] - (self.camera_height/2.0) #y-axis is inverted??? Works with arducopter

		 				#convert target location to angular radians
			 			x_angle = x_pixel * (self.camera_hfov / self.camera_width) * (math.pi/180.0)
			 			y_angle = y_pixel * (self.camera_vfov / self.camera_height) * (math.pi/180.0)

			 			#send commands to autopilot
			 			veh_control.report_landing_target(x_angle, y_angle, results[2],0,0)


		 	else:
		 			VN_logger.text(VN_logger.GENERAL, 'Not in landing mode')





	 	#terminate program
	 	VN_logger.text(VN_logger.GENERAL, 'Vehicle disconnected, Program Terminated')
	 	if(self.simulator == False):
	 		VN_video.stop_capture()




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
