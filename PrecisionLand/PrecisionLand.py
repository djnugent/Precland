#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
import cv2
import Queue

#COMMOM IMPORTS
from VisNav.Common.VN_config import VN_config
from VisNav.Common.VN_video import VN_video
from VisNav.Common.VN_dispatcher import VN_dispatcher
from VisNav.Common.VN_logger import VN_logger
from VisNav.Common.VN_util import *
from VisNav.Common.VN_position_vector import PositionVector
from VisNav.Common.VN_vehicle_control import veh_control

#PRECISION LAND IMPORTS
from VisNav.PrecisionLand.PL_gui import PrecisionLandGUI as gui
from VisNav.PrecisionLand.PL_sim import sim
from VisNav.PrecisionLand.CircleDetector import CircleDetector

#DRONEAPI IMPORTS
from droneapi.lib import VehicleMode, Location, Attitude


'''
Temporary Changes:
-added kill_camera(commented out)
'''


'''
TODO:
-Output angles from CircleDetector
-update config file
Future:


Improvements:
-fix config file naming/loading(make it more intelligent)
-add better target_detected logic(multiple frames required for a lock)
-add update rate to VN_logger
-fix project file structure
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

		#use simulator
		self.simulator = VN_config.get_boolean('simulator','use_simulator',True)

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
			sim.set_target_location(veh_control.get_home())
			#sim.set_target_location(Location(0,0,0))

		else:
			VN_video.start_capture(self.camera_index)


		#create an image processor
		detector = CircleDetector()

		#create a queue for images
		imageQueue = Queue.Queue()


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
			if self.always_run or (veh_control.get_mode() == "LAND") or (veh_control.get_mode() == "RTL"):

		 		#update how often we dispatch a command
		 		VN_dispatcher.calculate_dispatch_schedule()

		 		'''
		 		#get info from autopilot
		 		location = Location(0.000009,0,location.alt)
		 		attitude = Attitude(0,0,0)
		 		'''

		 		#update simulator
		 		if(self.simulator):
		 			#get info from autopilot
		 			location = veh_control.get_location()
		 			attitude = veh_control.get_attitude()
		 			sim.refresh_simulator(location,attitude)

		 		# grab an image
				capStart = current_milli_time()
				frame = self.get_frame()
				capStop = current_milli_time()

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
		 			VN_logger.text(VN_logger.AIRCRAFT,attitude)
		 			VN_logger.text(VN_logger.AIRCRAFT,location)

		 			#send commands to autopilot
		 			self.report_to_autopilot(results)

		 	else:
		 			VN_logger.text(VN_logger.GENERAL, 'Not in landing mode')
		 			




	 	#terminate program
	 	VN_logger.text(VN_logger.GENERAL, 'Vehicle disconnected, Program Terminated')
	 	if(self.simulator == False):
	 		VN_video.stop_capture()


	#get_frame - pull an image from camera or simulator
	def get_frame(self):
		if(self.simulator):
			return sim.get_frame()
		else:
			return VN_video.get_image()


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








