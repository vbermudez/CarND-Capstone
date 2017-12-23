from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        result = TrafficLight.UNKNOWN
        output = image.copy()
        red = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_r = np.array([0, 50, 50])
        upper_r = np.array([10, 255, 255])
        red_1 = cv2.inRange(red, lower_r, upper_r)
        lower_r = np.array([170, 50, 50])
        upper_r = np.array([180, 255, 255])
        red_2 = cv2.inRange(red, lower_r, upper_r)
        conv_img = cv2.addWeighted(red_1, 1., red_2, 1., 0.)
        blur_img = cv2.GaussianBlur(conv_img, (15, 15), 0)
        circles = cv2.HoughCircles(blur_img, cv2.HOUGH_GRADIENT, .5, 41, 
                    param1=70,
                    param2=30,
                    minRadius=5,
                    maxRadius=150)
        found = False
        if circles is not None:
            result = TrafficLight.RED
        return result, output
