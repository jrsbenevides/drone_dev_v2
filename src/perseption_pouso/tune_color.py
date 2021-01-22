import sys
import cv2
import numpy as np
import imutils

def color_filter_single():
	for i in range(1,3):
		filename = "image" + str(i) + ".jpg"
		cv_image = cv2.imread(filename)
		tune_color_filter(cv_image,BGRvalue)

def color_filter_video():
	video_stream = cv2.VideoCapture('video2_tubo.mp4')

	while video_stream.isOpened():
		ret, cv_image = video_stream.read()
		if ret == True:
			tune_color_filter(cv_image,BGRvalue)

			if cv2.waitKey(25) & 0xFF == ord('q'):
				break
		if cv2.waitKey(25) & 0xFF == ord('e'):
			break

class tune_color_filter():
	def __init__(self,cv_image,BGRvalue):
		self.cv_image = cv_image
		self.BGRvalue = BGRvalue

		self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV) # To HSV

		cv2.imshow('image',self.cv_image)
		cv2.setMouseCallback('image',self.get_points) # Recebe apenas ao doble-click
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		print(np.mean(self.BGRvalue, axis=0))
		print(np.std(self.BGRvalue, axis=0))

	def get_points(self,event,x,y,flags,param):
		if event == cv2.EVENT_LBUTTONDBLCLK:
			print((self.cv_image[y,x]))
			self.BGRvalue.append([int(self.cv_image[y,x][0]), int(self.cv_image[y,x][1]), int(self.cv_image[y,x][2])])
			self.cv_image = cv2.circle(self.cv_image, (x,y), 2, (0,0,255),-1)
			cv2.imshow('image',self.cv_image)

def save_frame(cv_image):
	global i
	i += 1
	if i%5 == 0 and i > 180:
		filename = "image" + str(i) + ".jpg"
		print(filename)
		cv2.imwrite(filename, cv_image)



def start_detector_video():
	camera_control_angle = 80.0
	bebop_z = 2.5

	
	# video_stream = cv2.VideoCapture('teste_1_1310.avi')
	while video_stream.isOpened():
		ret, cv_image = video_stream.read()
		if ret == True:
			cv_image,filtered,center,center_rotation = landing_detector_main(cv_image,camera_control_angle,bebop_z)
			cv2.imshow('Frame',cv_image)
			# cv2.imshow('Filtered',filtered)

			if cv2.waitKey(25) & 0xFF == ord('q'):
				break

# # # # # # #
def tune_color_filter_selector():
	color_filter_single()
	# color_filter_video()

global i
i = 0

global BGRvalue
BGRvalue = []

tune_color_filter_selector()
	



