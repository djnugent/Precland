import sys
#sys.path.remove('/usr/bin')
import usb.core, usb.util
import numpy as np

#Make the PX4Flow sensor behave as a webcam
#Thanks to Kevin Mehall for his contribution
class Flow_Camera:
 	def __init__(self):
 		self.size = 320
 		image = np.zeros((self.size, self.size), dtype='uint8')
 		try:
 			self.dev = usb.core.find(idVendor=0x26ac, idProduct=0x0015)
			self.endpoint = self.dev[0][(2,0)][0]
		except:
			pass

 	def isOpened(self):
 		return (self.dev is not None) and (self.endpoint is not None)

 	def read(self):
 		data = self.endpoint.read(64 * (1 + (self.size*self.size) / 64), timeout=2000)
 		try:
 			image = np.frombuffer(data, dtype='uint8').reshape(self.size, self.size)
 		except ValueError: # we usually fail on our first images
 			image = np.zeros((self.size, self.size), dtype='uint8')

 		return (True, image)


# create a single global object
flow_cam = Flow_Camera()

if __name__ == "__main__":
	import cv2
	if flow_cam.isOpened():

		while True:
			ret , img = flow_cam.read()
			cv2.imshow("flow",img)
			cv2.waitKey(1)
	else:
		print "No camera found!!"





