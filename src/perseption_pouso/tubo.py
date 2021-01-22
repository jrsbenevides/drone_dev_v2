import sys
import cv2
import numpy as np
import imutils

def save_frame(cv_image):
	global i
	i += 1
	if i%1 == 0 and i > 1:
		filename = "image" + str(i) + ".jpg"
		# print(filename)
		cv2.imwrite(filename, cv_image)

# # # # # # #
def start_detector_singleImage():
	camera_control_angle = 80.0
	bebop_z = 2.5
	for i in range(1,3):
		# i = 1
		# filename = "image" + str(i) + ".jpeg"
		filename = "image" + str(i) + ".jpg"
		# print(filename)
		cv_image = cv2.imread(filename)
		
		cv_image,marker = tubo_detector_main(cv_image)

def start_detector_video():
	camera_control_angle = 80.0
	bebop_z = 2.5

	video_stream = cv2.VideoCapture('video2_tubo.mp4')
	# video_stream = cv2.VideoCapture('teste_1_1310.avi')
	while video_stream.isOpened():
		ret, cv_image = video_stream.read()
		if ret == True:
			cv_image,marker = tubo_detector_main(cv_image)
			if marker == 0:
				print("Verde")
			cv2.imshow('Frame',cv_image)
			# cv2.imshow('Filtered',filtered)

			if cv2.waitKey(25) & 0xFF == ord('q'):
				break
 

# # # # # # #

def tubo_detector_main(cv_image):
	global height
	global width
	marker = 10
	height, width, _ = cv_image.shape
	cv_image = imutils.rotate(cv_image, 180)
	filtered = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # To HSV

	if tube_on_image(cv_image):
		cv_image,marker = carmona_epic_fun(cv_image,filtered)
		# cv_image,filtered,center,center_rotation = get_contours(cv_image,filtered,camera_control_angle,bebop_z)

	return(cv_image,marker)

def tube_on_image(cv_image):
	mean_value = [ 153.28682171, 146.62790698, 181.58139535]
	std_value = [ 61.63083314, 18.22047139, 18.97640219]

	minmean = np.array([mean_value[0] - std_value[0], mean_value[1] - std_value[1], mean_value[2] - std_value[2]])
	maxmean = np.array([mean_value[0] + std_value[0], mean_value[1] + std_value[1], mean_value[2] + std_value[2]])

	cv_image = cv2.inRange(cv_image, minmean, maxmean)
	# print(np.sum(cv_image))
	if np.sum(cv_image) > 4000:
		return(True)
	else:
		return(False)

def carmona_epic_fun(cv_image,filtered):
	marker = 10

	_, saturationThres = cv2.threshold(filtered[:,:,1],0,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)

	size = 10
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size,size))
	kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT,(size,size))
	saturationThres = cv2.morphologyEx(saturationThres, cv2.MORPH_CLOSE, kernel)
	saturationThres = cv2.morphologyEx(saturationThres, cv2.MORPH_GRADIENT, kernel2)

	saturationThres = cv2.Canny(saturationThres, 100, 100, 7)
	cnts = cv2.findContours(saturationThres, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	for c in cnts:
		if cv2.contourArea(c) > 100.0 and cv2.arcLength(c,True) < 1000.0:
			mask = np.zeros((height,width,3), np.uint8)
			rect = cv2.minAreaRect(c)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			mask = cv2.drawContours(mask,[box],0,(255,255,255),-1)
			maskimage = cv2.bitwise_and(cv_image, mask)
			# print("verde: "+ str(np.sum(maskimage[:,:,1])))
			# print("vermelho: "+ str(np.sum(maskimage[:,:,2])))
			if marcador_verde(maskimage):
				cv_image = cv2.drawContours(cv_image,[box],0,(0,255,255),3)
				# cv_image = cv2.putText(cv_image, "Verde", (0,255,0))
				marker = 0
			# cv2.imshow('GREEN',maskimage)
	
	return(cv_image,marker)

def marcador_verde(cv_image):
	s = 2
	mean_value = [ 31.44444444, 65.22222222, 92.77777778]
	std_value = [ 24.94933137*s, 36.83831875*s, 20.51437316*s]

	minmean = np.array([mean_value[0] - std_value[0], mean_value[1] - std_value[1], mean_value[2] - std_value[2]])
	maxmean = np.array([mean_value[0] + std_value[0], mean_value[1] + std_value[1], mean_value[2] + std_value[2]])

	cv_image = cv2.inRange(cv_image, minmean, maxmean)
	if np.sum(cv_image) > 100.0 and np.sum(cv_image) < 100000.0:
		return(True)
	else:
		return(False)


def landing_detector_cyan(cv_image):
	s = 4

	mean_value = [ 153.28682171, 146.62790698, 181.58139535]
	std_value = [ 70.63083314, 35.22047139, 35.97640219]
	# s = 8
	# mean_value = [ 22, 250, 236] # HSV yrllow
	# std_value = [5*s, 5*s, 5*s] # HSV yrllow

	minmean = np.array([mean_value[0] - std_value[0], mean_value[1] - std_value[1], mean_value[2] - std_value[2]])
	maxmean = np.array([mean_value[0] + std_value[0], mean_value[1] + std_value[1], mean_value[2] + std_value[2]])

	cv_image = cv2.inRange(cv_image, minmean, maxmean)

	# Opening
	# size = 10
	# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size,size))
	# cv_image = cv2.dilate(cv_image, kernel, iterations = 1)
	# cv_image = cv2.erode(cv_image, kernel, iterations = 1)
	# tool_show_image(cv_image)
	return(cv_image)

# # # # # # #

def tool_show_image(cv_image):
	cv2.imshow('image',cv_image)
	cv2.waitKey(0)
	# cv2.destroyAqllWindows()

global i
i = 0

# start_detector_singleImage()

# start_detector_video()