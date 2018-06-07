import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image, TL_BB_list):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
	if not TL_BB_list:
		return TrafficLight.UNKNOWN
	
	xmin = TL_BB_list[0].xmin
	xmax = TL_BB_list[0].xmax
	ymin = TL_BB_list[0].ymin
	ymax = TL_BB_list[0].ymax
	
	bb_image = image[ymin:ymax, xmin:xmax]
	hsv_bb_img = cv2.cvtColor(bb_image, cv2.COLOR_BGR2HSV)


	# Red Color ranges
	frame_threshed_red1 = cv2.inRange(hsv_bb_img, (0, 70, 50), (10, 255, 255))
	frame_threshed_red2 = cv2.inRange(hsv_bb_img, (170, 70, 50), (180, 255, 255))

	# Yellow Color ranges
	frame_threshed_yellow = cv2.inRange(hsv_bb_img, (40.0/360*255, 100, 100), 
(66.0/360*255, 255, 255))
	# Green Color ranges
	frame_threshed_green = cv2.inRange(hsv_bb_img, (90.0/360*255, 100, 100), (140.0/360*255, 255, 255))

	# If more than a certain number of pixels are red
	if cv2.countNonZero(frame_threshed_red1) + cv2.countNonZero(frame_threshed_red2) >  40:
		print('Red Light Detected!')
		return TrafficLight.RED
	elif cv2.countNonZero(frame_threshed_yellow) > 20 :
 		print('Yellow Light Detected!')
		return TrafficLight.YELLOW
	elif cv2.countNonZero(frame_threshed_green) > 20 :
 		print('Green Light Detected!')
		return TrafficLight.GREEN
	else:
		print('Warning! Could not determine color of the light!')
		return TrafficLight.UNKNOWN

