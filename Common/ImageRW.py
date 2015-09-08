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

		filename = self.dir + "/" + str(self.frame) + '.bmp'
		cv2.imwrite(filename,img)
		self.frame += 1


class ImageReader():
	def __init__(self,filename,fps, start = 0, stop = 0, loop = False):
		self.frame = start
		self.dir = filename
		self.fps = fps
		self.start = start
		self.stop = stop
		self.loop = loop

		if not os.path.exists(self.dir):
			print "no file"
			sys.exit(0)

	def read(self):
		filename = self.dir + "/" + str(self.frame) + '.bmp'
		img = cv2.imread(filename)

		if img is None or self.frame > self.stop:
			#Loop footage
			if self.loop == True and self.frame > self.start:
				self.frame = self.start
				print 'loop'
				filename = self.dir + "/" + str(self.frame) + '.bmp'
				img = cv2.imread(filename)
			else:
				img = None
		print self.frame
		self.frame += 1
		time.sleep(1.0/self.fps)
		return img

	def peek(self, frame_num):
		filename = self.dir + "/" + str(frame_num) + '.bmp'
		img = cv2.imread(filename)
		return img

	def convert_to_video(self):
		self.loop = False

		img = self.peek(0) # read first image to get dimensions
		height = img.shape[0]
		width = img.shape[1]

		ex = int(cv2.cv.CV_FOURCC('M','J','P','G'))
		video_writer = cv2.VideoWriter(self.dir + '.avi', ex, 2, (width,height))

		while True:
			img = self.read()
			if img is not None:
				video_writer.write(img)
			else:
				print "no images found, done"
				sys.exit()

	def reset(self):
		self.frame = self.start


if __name__ == "__main__":
	cam = ImageReader("/home/daniel/Videos/Smart_Camera-gui-4", 1000, start = 1780, stop = 1855, loop = False)
	cam.convert_to_video()
	'''
	ex = int(cv2.cv.CV_FOURCC('M','J','P','G'))
	video_writer = cv2.VideoWriter('/home/daniel/Test_footage2/sample1.avi', ex, 10, (320,320))

	while True:

		img = cam.read()
		if img is not None:

			video_writer.write(img)
			cv2.imshow('img',img)
			cv2.waitKey(1)
		else:
			print "no images found"
			sys.exit()
	'''
