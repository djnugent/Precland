#!/usr/bin/python
import cv2
import numpy as np
import os

#COMMOM IMPORTS
from VisNav.Common.VN_config import VN_config




class PrecisionLandTargetGenerator():


	def __init__(self):
		self.ring_color = (0,0,0)
		self.background_color = (255,255,255)
		self.img_size = 0
		self.ratio = np.zeros(10,np.float)
		self.length = 0




	def draw_ring(self, outer_radius, inner_radius):
		for radius in range(inner_radius,outer_radius):
			if(radius != 0):
				cv2.circle(self.img, (self.img_size/2,self.img_size/2), radius, self.ring_color, thickness=2, lineType=cv2.CV_AA, shift=0)


	def export_to_config_file(self):
		#trim off excess data
		cnt = 0
		for r in self.ratio:
			if r != 0:
				cnt += 1
		trim_ratio = np.resize(self.ratio,cnt)

		sc_config.config.set_array('algorithm','target_code',trim_ratio)
		sc_config.config.save()

	def import_from_config_file(self):
		target_code_def = np.array([0.8,0.91,0.76,0.84,0.7,0.66,0.49])
		raw_ratio = sc_config.config.get_array('algorithm', 'target_code',target_code_def) 

		for i in range(0,len(self.ratio)):
			#update buffer
			if(i < len(raw_ratio)):
				self.ratio[i]= raw_ratio[i]
			else:
				self.ratio[i] = 0

			#update trackbars
			field = 'Ratio ' + str(i)
			cv2.setTrackbarPos(field,'parameters',int(self.ratio[i] * 100))


	def save_image(self):
		path = 'target.jpg'
		#get file valid filename
		i = 0
		while os.path.exists(path):
		    path = 'target[{0}].jpg'.format(i)
		    i+=1

		cv2.imwrite(path,self.img)

	def main(self):


		def updateParams(x):
			pass

		cv2.namedWindow('parameters')

		# create trackbars for parameters
		cv2.createTrackbar('Image size','parameters',800,6000,updateParams)
		cv2.createTrackbar('Border','parameters',5,200,updateParams)
		cv2.createTrackbar('Ratio 0','parameters',80,100,updateParams)
		cv2.createTrackbar('Ratio 1','parameters',91,100,updateParams)
		cv2.createTrackbar('Ratio 2','parameters',76,100,updateParams)
		cv2.createTrackbar('Ratio 3','parameters',84,100,updateParams)
		cv2.createTrackbar('Ratio 4','parameters',70,100,updateParams)
		cv2.createTrackbar('Ratio 5','parameters',66,100,updateParams)
		cv2.createTrackbar('Ratio 6','parameters',49,100,updateParams)
		cv2.createTrackbar('Ratio 7','parameters',0,100,updateParams)
		cv2.createTrackbar('Ratio 8','parameters',0,100,updateParams)
		cv2.createTrackbar('Ratio 9','parameters',0,100,updateParams)



		while True:
			#create image
			self.img_size = cv2.getTrackbarPos('Image size','parameters')
			self.img = np.zeros((self.img_size,self.img_size,3), np.uint8)
			self.img[:] = self.background_color

			border = cv2.getTrackbarPos('Border','parameters')


			#grab ring data
			self.ratio[0] = cv2.getTrackbarPos('Ratio 0','parameters') /100.0
			self.ratio[1] = cv2.getTrackbarPos('Ratio 1','parameters') /100.0
			self.ratio[2] = cv2.getTrackbarPos('Ratio 2','parameters') /100.0
			self.ratio[3] = cv2.getTrackbarPos('Ratio 3','parameters') /100.0
			self.ratio[4] = cv2.getTrackbarPos('Ratio 4','parameters') /100.0
			self.ratio[5] = cv2.getTrackbarPos('Ratio 5','parameters') /100.0
			self.ratio[6] = cv2.getTrackbarPos('Ratio 6','parameters') /100.0
			self.ratio[7] = cv2.getTrackbarPos('Ratio 7','parameters') /100.0
			self.ratio[8] = cv2.getTrackbarPos('Ratio 8','parameters') /100.0
			self.ratio[9] = cv2.getTrackbarPos('Ratio 9','parameters') /100.0

			outer_radius = 0
			inner_radius = 0
			i = 0

			#draw rings
			while(i < len(self.ratio)):
				#outer most radius
				if(i == 0):
					outer_radius = (self.img_size-border)/2
				else:
					outer_radius = int(inner_radius * (self.ratio[i]))
					i+=1

				#inner most radius
				if(i >= len(self.ratio)):
					inner_radius = 0
				else:
					inner_radius = int(outer_radius * (self.ratio[i])) 
				
				self.draw_ring(outer_radius,inner_radius)
				i +=1


			cv2.imshow('Target', self.img)

			#user input
			k = cv2.waitKey(33) & 0xFF
			if k == ord('e'):
				self.export_to_config_file()
				print "Exported target data"
			if k == ord('i'):
				self.import_from_config_file()
				print "Imported target data"
			if k == ord('s'):
				self.save_image()
				self.export_to_config_file()
				print "Saved image and exported target data"



if __name__ == '__main__':
	targetGen = PrecisionLandTargetGenerator()
	targetGen.main()