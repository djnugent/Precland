import os
import cv2
import time
import sys


class ImageWriter():

	def __init__(self, filename):

		self.frame = 0
		self.dir = filename
		if not os.path.exists(self.dir):
			os.makedirs(self.dir)

	def write(self, img):

		filename = self.dir + "/" + str(self.frame) + '.jpg'
		cv2.imwrite(filename,img)
		self.frame += 1


class ImageReader():
	def __init__(self,filename,fps, loop = False):
		self.frame = 0
		self.dir = filename
		self.fps = fps
		self.loop = loop

		if not os.path.exists(self.dir):
			print "no file"
			sys.exit(0)

	def read(self):
		filename = self.dir + "/" + str(self.frame) + '.jpg'
		img = cv2.imread(filename)

		if self.loop == True and img is None and self.frame > 0:
			self.frame = 0
			print 'loop'
			filename = self.dir + "/" + str(self.frame) + '.jpg'
		self.frame += 1
		time.sleep(1.0/self.fps)
		return img

if __name__ == "__main__":
	cam = ImageReader("/home/daniel/visnav/vids/Smart_Camera-gui-1", 90, loop = True)
	while True:
		img = cam.read()
		cv2.imshow('img',img)
		cv2.waitKey(1)
