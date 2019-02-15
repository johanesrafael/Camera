#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from motor_driver import MotorDriver
from std_srvs.srv import Empty, EmptyRequest

class RobotMover(object):

    def __init__(self):
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.motor_driver = MotorDriver()
        rospy.wait_for_service('/raspicam_node/start_capture')

        start_cam = rospy.ServiceProxy('/raspicam_node/start_capture', Empty)
        request_e = EmptyRequest()
        start_cam(request_e)
        rospy.loginfo("Started Camera")


    def cmd_vel_callback(self, msg):
        forward_speed = msg.linear.x
        turn_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.set_speed(forward_speed)

        # Decide Direction to move
        if turn_speed > 0:
            self.motor_driver.right()
        elif turn_speed < 0:
            self.motor_driver.left()
        else:
            if forward_speed > 0:
                self.motor_driver.forward()
            elif forward_speed < 0:
                self.motor_driver.reverse()
            else:
                self.motor_driver.stop()

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('morpheuschair_cmd_vel_listener', anonymous=True)
    robot_mover = RobotMover()
    robot_mover.listener()