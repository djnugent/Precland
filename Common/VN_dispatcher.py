#!/usr/bin/python
#SYSTEM IMPORTS
import multiprocessing
import numpy as np
import time

#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_logger import VN_logger
from Common.VN_util import *

'''
VisNavDispatcher
This class helps process images across multicores
It can be used on (n) number of cores: including a single core.
The disatcher sends images out for processing as fast as the camera can feed the images or as fast as the CPU can process them
In addition to compensating to the number of core available for use, the program compensates for varying operating conditions
	i.e. slower image capture in low light settings; longer processing time with 'busy' images
	It will take at least 4 cycles for the program to stabilize its condition
The dispatcher may not use all the cores it has access to. This is common for functions that already run 'fast'

Usage:
Upon intializing the dispatcher, the function/process to dispatched must be passed as an arguement to the constructor
The first parameter of the function/process must be a child pipe connection
	This pipe connection gets passed in AUTOMATICALLY by the dispatcher.
	Meainng it is never used outside the function definition and the dispatcher class
All other arguements depend on the operation of the function/process
	These will be passed into the dispatch() method
	Exculding the pipe connection. The dispatcher passes that in on its own
The function/process will have to return all results through a pipe.
Results must be returned as a tuple
The first result of the tuple MUST BE an interal run time(in millis) of the function/process
'''




class VisNavDispatcher(object):

	def __init__(self):

		#The number of core to be used while processing image data
		#This number may be less than the actaul number of cores on the CPU depending on the users specifications
		#cores = min(desiredCores, multiprocessing.cpu_count()) //dont allow more cores than the CPU has available and don;t run more than the user wants
		desired_cores = VN_config.get_integer('processing', 'desired_cores', 4)
		available_cores = min(desired_cores, multiprocessing.cpu_count())
		#check if a core is already in use for background image capture
		cores_image_capture = int(VN_config.get_boolean('processing','background_capture', True))
		self.cores_processing = max(available_cores - cores_image_capture,1)


		#The time(in millis) is takes to capture an Image
		#Frame rate = 1000/captureTime
		#****On some cameras frame rate is dependent on camera exposure level
		#****i.e. dark = slower fps and light = fast frame rate
		#****Capture time is dependent on available CPU
		#This time will be dynamically calculated using an rolling average
		self.captureTime = 0
		self.captureTimeSet = np.zeros(4)

		#The time(in millis) it takes to process an image
		#****Process time is dependent on available CPU and the image
		#This time will be dynamically calculated using an rolling average
		self.processTime = 0
		self.processTimeSet = np.zeros(4)

		#How often a image is dispatched to be processed(in millis)
		#Determined by splitting up processTime among available number of cores
		#*****This will not be smaller than captureTime because we won't allow it to process frames more often than capturing frames; it will cause sync issues with Pipe()
		#*****If the CPU is slow and has limited cores, we may process everyother frame or every nth frame!!!
		#runTime = max(processTime/processing_cores,captureTime)
		self.runTime = 0

		#set up a pipe to pass info between background processes and dispatcher
		self.parent_conn, self.child_conn = multiprocessing.Pipe()

		#last time we started to process an image
		self.lastDispatch = 0

		#last time an image process completed
		self.lastRetreival = 0



	#calculate_dispatch_schedule - 	calculate the schedule of the image processing
	def calculate_dispatch_schedule(self):
		self.runTime = max(self.processTime/(self.cores_processing * 1.0), self.captureTime)

	#update_capture_time - updates the time it takes the camera to capture an image
	def update_capture_time(self,cap_time):
		#update captureTime
		self.captureTime, self.captureTimeSet = self.rolling_average(self.captureTimeSet, cap_time)

	#is_ready - checks whether it is time to dispatch a new process
	def is_ready(self):
		return (time.time() * 1000 - self.lastDispatch >= self.runTime)

	#dispatch - dispatch a new process to a core
	def dispatch(self, target, args):
		
		if self.is_ready() == False:
			return

		#mark our last dispatch time
		self.lastDispatch = time.time() * 1000


		#splice together arguements
		#args = (args[0],) + (self.child_conn,) + args[1:]
		args = (self.child_conn,) + args

		#create a process to run in background
		p = multiprocessing.Process(target=target, args = args)
		p.daemon=True
   		p.start()

	#is_available - checks if a process has completed and results are ready
	def is_available(self):
		#check to see if a process has finished and sent data
		return self.parent_conn.poll()

	#retreive - returns the results of the most recent completed process
	def retreive(self):
		if self.is_available():

			#grab results
			results = self.parent_conn.recv()

			#update processTime. All processes must return a runtime through the pipe
			self.processTime, self.processTimeSet = self.rolling_average(self.processTimeSet, results[1])

			#Calculate real runtime. Diagnostic purposes only
			#In an ideal system the dispatch rate would equal the retreival rate. That is not the case here
			actualRunTime = time.time() * 1000 - self.lastRetreival
			self.lastRetreival = time.time() * 1000

			VN_logger.text(VN_logger.PERFORMANCE, "DispatchHz: {0} runHz: {1} ProcessHz: {2} CaptureHz: {3} Processes: {4} ".format(round(1000.0/(self.runTime)), round(1000.0/actualRunTime), round((1000.0/results[1])),round(1000.0/self.captureTime), len(multiprocessing.active_children())))
			return results

		return None



	#rolling_average - returns a rolling average of a data set and inputs a new value
	def rolling_average(self, dataSet, newValue):
		total = newValue
		for i in range(0, len(dataSet)-1):
			dataSet[i] = dataSet[i+1]
			total += dataSet[i+1]
		dataSet[len(dataSet) - 1] = newValue
		return total * 1.0/len(dataSet) , dataSet

	#test for SmartCameraDispatcher
	def main(self):

		#Simulated load
		def dummyLoad(conn,load):
 			time.sleep(load/1000.0)
 			result = (load,)
 			conn.send(result)


	 	while True:

	 		#update how often we dispatch a command
	 		self.calculate_dispatch_schedule()

	 		# simulate Retreiving an image
			capStart = time.time() * 1000
			time.sleep(33.3/1000) #img = smartCam.get_image()
			capStop = time.time() * 1000

	 		#update capture time
	 		self.update_capture_time(capStop-capStart)


			#Process image
			#We schedule the process as opposed to waiting for an available core
			#This brings consistancy and prevents overwriting a dead process before
			#information has been grabbed from the Pipe
			if self.is_ready():
				self.dispatch(target=dummyLoad, args=(83.3))

	 		#retreive results
	 		if self.is_available():
	 			results = self.retreive()






# create a single global object
VN_dispatcher = VisNavDispatcher()

if __name__ == "__main__":
	VN_dispatcher.main()
