import sys
import cv2
import numpy as np
import imutils


class tune_color_filter():
	def __init__(self):
		for i in range(1,2):
			i = 11
			filename = "image" + str(i) + ".jpg"
			self.cv_image = cv2.imread(filename)
			self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV) # To HSV
			self.BGRvalue = []

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

# # # # # # #

def save_frame(cv_image):
	global i
	i += 1
	if i%10 == 0 and i > 180:
		print(i)
		filename = "image" + str(i) + ".jpg"
		cv2.imwrite(filename, cv_image)

# # # # # # #
def start_detector_singleImage():
	camera_control_angle = 30.0
	bebop_z = 2.5

	for i in range(5,9):
		i = 33
		path = "~/Pictures/"
		filename = path + "image" + str(i) + ".jpg"
		# filename = "image" + str(i) + ".jpg"
		print(filename)
		image = cv2.imread(filename)
	
		landing_detector_main(image,camera_control_angle,bebop_z)

def start_detector_video():

	video_stream = cv2.VideoCapture('video1.avi')

	while video_stream.isOpened():
		ret, cv_image = video_stream.read()
		if ret == True:
			cv_image,filtered,center,center_rotation = landing_detector_main(cv_image)
			cv2.imshow('Frame',cv_image)
			cv2.imshow('Filtered',filtered)

			if cv2.waitKey(25) & 0xFF == ord('q'):
				break
 
# # # # # # #

def landing_detector_main(cv_image,camera_control_angle,bebop_z):

	# filtered = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # To HSV
	# filtered = landing_detector_cyan(filtered)

	filtered = carmona_epic_fun(cv_image)

	cv_image,filtered,center,center_rotation = get_contours(cv_image,filtered,camera_control_angle,bebop_z)
	return(cv_image,filtered,center,center_rotation)

def carmona_epic_fun(cv_image):

	_, redThres = cv2.threshold(cv_image[:,:,2],0,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)
	_, greenThres = cv2.threshold(cv_image[:,:,1],0,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)
	_, blueThres = cv2.threshold(cv_image[:,:,0],0,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)

	# cv2.imshow('RED',redThres)
	# cv2.imshow('GREEN',greenThres)
	# cv2.imshow('BLUE',blueThres)
	blue_imgage = cv2.bitwise_and(blueThres, cv2.bitwise_not(redThres))
	yellow_imgage = cv2.bitwise_and(cv2.bitwise_and(redThres,greenThres), cv2.bitwise_not(blueThres))

	# # Opening
	size = 10
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size,size))
	blue_imgage = cv2.dilate(blue_imgage, kernel, iterations = 1)
	blue_imgage = cv2.erode(blue_imgage, kernel, iterations = 1)
	return(blue_imgage)
	# Draw contours
	# blue_imgage = cv2.Canny(blue_imgage, 100, 100, 7) # 100,100,7
	# bnts = cv2.findContours(blue_imgage, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	# bnts = imutils.grab_contours(bnts)
	# blue_imgage = image_mask(cv_image,bnts)
	# blue_imgage = cv2.drawContours(blue_imgage, bnts, -1, (255, 255, 255), -1)

	cv2.imshow('BLUE',blue_imgage)
	cv2.imshow('YELLOW',yellow_imgage)

def landing_detector_cyan(cv_image):
	s = 4
	mean_value = [ 102, 230, 192] # HSV 2&1
	std_value = [2*s, 15*s, 11*s] # HSV 2&1

	# mean_value = [ 106, 255, 210] # HSV 2&1
	# std_value = [2*s, 30*s, 20*s] # HSV 2&1

	minmean = np.array([mean_value[0] - std_value[0], mean_value[1] - std_value[1], mean_value[2] - std_value[2]])
	maxmean = np.array([mean_value[0] + std_value[0], mean_value[1] + std_value[1], mean_value[2] + std_value[2]])

	cv_image = cv2.inRange(cv_image, minmean, maxmean)

	# Opening
	size = 10
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size,size))
	cv_image = cv2.dilate(cv_image, kernel, iterations = 1)
	cv_image = cv2.erode(cv_image, kernel, iterations = 1)

	return(cv_image)

def get_contours(cv_image,filtered,camera_control_angle,bebop_z):
	center = []
	center_rotation = []
	height, width, _ = cv_image.shape

	# filtered = cv_image
	
	filtered = cv2.Canny(filtered, 100, 200, 7) # 100,100,7
	# filtered = cv2.blur(filtered,(1,1))
	cnts = cv2.findContours(filtered.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	filtered = cv2.drawContours(filtered, cnts, -1, (255, 255, 255), -1)
	
	filtered = cv2.Canny(filtered, 100, 100, 7) # 100,100,7

	cnts = cv2.findContours(filtered.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	cnts = sorted(cnts, key=cv2.contourArea)

	for c in cnts:
		if cv2.contourArea(c) > 5000:
			center_point = contourCenter(c)

			center_rotation.append(get_rotation(center_point,camera_control_angle,bebop_z))
			center.append(center_point)

	for point in center:
		cv_image = cv2.line(cv_image, (point[0],242), (point[0],point[1]), (255,0,0), 2) # X
		cv_image = cv2.line(cv_image, (302,242), (point[0],242), (0,0,255), 2) # Y
		cv_image = cv2.line(cv_image, (302,242), (point[0],point[1]), (0,255,0), 2) # XY
	return(cv_image,filtered,center,center_rotation)

def contourCenter(c):
	M = cv2.moments(c)
	return(int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), cv2.contourArea(c))

def get_rotation(center,camera_control_angle,bebop_z): # Need hardcode Kparam
    center_rotation = (0,0,0)
    # Kparam = [595.0354940047112, 0.0, 301.82356774120103, 0.0, 594.6744984701004, 242.97859777091915, 0.0, 0.0, 1.0] # Camera de baixo
    Kparam = [396.17782, 0.0, 322.453185, 0.0, 399.798333, 174.243174, 0.0, 0.0, 1.0] # Bebop
    
    pitch = -np.arctan2((center[1] - Kparam[5]),Kparam[4]) # atan2((Y - v0)/fy)
    yaw =  np.arctan2((center[0] - Kparam[2]),Kparam[0]) # atan2((X - u0)/fx)
    
    # print("Pitch y[rad] = " + str(pitch))

    phi = np.deg2rad(camera_control_angle)

    X = bebop_z / np.tan(phi)

    u = X * np.tan(pitch)
    if center[1] >= Kparam[5]:

    	dist = X + u/np.sin(pitch - phi)
    else:
    	dist = X - u/np.sin(pitch - phi)

    # print("Yaw x[deg] = " + str(np.rad2deg(yaw)))
    # print("Distance [m] = " + str(dist) + "\n")

    center_rotation = (pitch,yaw,dist)
    return(center_rotation)


# # # # # # #

def tool_show_image(cv_image):
	cv2.imshow('image',cv_image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

global i
i = 0

# start_detector_singleImage()

# start_detector_video()

# tune_color_filter()
	



