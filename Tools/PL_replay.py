import cv2
import numpy
import os
import sys
import argparse



class SmartCameraReplay():


	def __init__(self):
		pass

	def start(self):

		#parse arguments
		parser = argparse.ArgumentParser(description="Replay video and text logs from a flight")
		#required arguments
		parser.add_argument('log_file', action="store", type=str, 
		 					help='Enter absolute location of log file. Ex: /home/odroid/Smart_Camera/logs/Smart_Camera-2015-01-22-20-35.log')
		parser.add_argument('raw_vid', action="store", type=str, 
		 					help='Enter absolute location of RAW video file. Ex: /home/odroid/Smart_Camera/logs/Smart_Camera-raw-2015-01-22-20-35.avi')
		#optional arguments
		parser.add_argument('-g' ,'--gui_vid' , action="store", type=str, 
		 					help='Enter absolute location of GUI video file. Ex: /home/odroid/Smart_Camera/logs/Smart_Camera-gui-2015-01-22-20-35.avi')

		args, unknown = parser.parse_known_args()


		#check and open files
		#log
		if(not os.path.exists(args.log_file)):
			print 'Unable to find log file'
			sys.exit(0)
		self.log = open(args.log_file)
		#raw video
		if(not os.path.exists(args.raw_vid)):
			print 'Unable to find raw video file'
			sys.exit(0)
		self.raw = cv2.VideoCapture(args.raw_vid)
		#gui video
		if(args.gui_vid is not None):
			if(not os.exists(args.log_file)):
				print 'Unable to find gui file'
				sys.exit(0)
			self.gui = cv2.VideoCapture(args.gui_vid)


	def run(self):
		log_text = self.log.readlines()
		for line in log_text:





class line():
	def __init__(self, time, level, msg):
		pass





if __name__ == '__main__':
	rep = SmartCameraReplay()
	rep.start()
	rep.run()