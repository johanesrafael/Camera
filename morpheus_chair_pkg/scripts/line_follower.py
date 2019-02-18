#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(object):
    def __init__(self, camera_topic="/raspicam_node/image_raw", cmd_vel_topic="/cmd_vel"):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    def camera_callback(self, data):

        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # We get image dimensions and crop the parts of the image we dont need
        # Bare in mind that because its image matrix first value is start and second value is down limit.
        # Select the limits so that it gets the line not too close, not too far and the minimum portion possible
        # To make process faster.
        # TODO: Get multiple lines so that we can generate paths.
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 100
        crop_img = cv_image[(height) / 2 + descentre:(height) / 2 + (descentre + rows_to_watch)][1:width]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        rospy.loginfo("Number of centroids==>" + str(len(contours)))
        centres = []
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            try:
                centres.append((int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])))
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

        rospy.loginfo(str(centres))
        # Select the right centroid
        # [(542, 39), (136, 46)], (x, y)
        most_right_centroid_index = 0
        index = 0
        max_x_value = 0

        centroids_detected = []

        for candidate in centres:
            # Retrieve the cx value
            cx = candidate[0]
            # Get the Cx more to the right
            if cx >= max_x_value:
                max_x_value = cx
                most_right_centroid_index = index
            index += 1

            try:
                cx = centres[most_right_centroid_index][0]
                cy = centres[most_right_centroid_index][1]
                rospy.logwarn("Winner ==" + str(cx) + "," + str(cy) + "")
            except:
                cy, cx = height / 2, width / 2

            centroids_detected.append([cx,cy])
            # Draw the centroid in the result image
            cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)


        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)

        cv2.waitKey(1)

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('line_follower_start', anonymous=True)
    robot_mover = LineFollower()
    robot_mover.loop()
