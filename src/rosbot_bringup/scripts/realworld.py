#!/usr/bin/env python

# Python libraries
import sys
import time

# NumPy and SciPy
import numpy as np
from scipy.ndimage import filters
import math

import imutils
from tf import transformations

# OpenCV
import cv2

# ROS libraries
import roslib
import rospy

# ROS Messages
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkState
from std_msgs.msg import Int32
from gazebo_msgs.msg import LinkStates

# Constants
pixel_limit = 170  # limit for stopping the robot
width_camera = 320  # dimension of the camera
lin_vel_move = 0.2  # linear velocity
ang_vel_move = 0.5  # angular velocity
no_vel_move = 0.0  # stop velocity
pixel_thr = 18  # threshold in pixels for alignment

# Controller's gains
kp_d = 0.2  # control distance gain
kp_a = 3.0  # control angular gain
ub_d = 0.3  # upper bound distance


# Class for image features
class image_feature:

    def __init__(self):
        '''Initialize ROS publisher, ROS subscriber'''
        rospy.init_node('image_feature', anonymous=True)

        # Topic where we publish velocity commands
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Publisher for the JointState message
        self.joint_state_pub = rospy.Publisher("/robot_exp/camera_velocity_controller/command", Float64, queue_size=1)

        # Subscriber for camera image
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.move_callback, queue_size=1)

        # Subscriber for marker ID
        self.subscriber = rospy.Subscriber("/id_publisher", Int32, self.id_callback, queue_size=1)

        # Subscriber for the marker center
        self.marker_center_sub = rospy.Subscriber("/marker_point", Point, self.marker_center_callback, queue_size=1)

        # Subscriber for the pixel side of the marker
        self.pixel_side_sub = rospy.Subscriber("/pixel_side_marker", Float64, self.pixel_callback, queue_size=1)

        # Retrieve marker list from parameter server
        self.marker_list = rospy.get_param('/marker_publisher/marker_list')

        # initialize variables
        self.marker_center_x = 0.0  # center x
        self.marker_center_y = 0.0  # center y
        self.marker_id = 0  # marker id
        self.current_pixel_side = 0.0  # pixel side of the marker

    def move_callback(self, ros_data):
        # Check if the marker list is empty
        if not self.marker_list:
            # Send a message to close marker_publisher on the topic where we write the marker number
            rospy.signal_shutdown("All markers reached")
            return

        # operation mode: camera fixed
        if self.marker_id == self.marker_list[0]:
            print("MARKER FOUND!")

            # Compute error
            self.error = abs(self.marker_center_x - width_camera) # Error between the marker's center and the camera's center

            if self.current_pixel_side > pixel_limit:
                # stop the robot when the marker is reached
                print("MARKER REACHED: " + str(self.marker_id))
                cmd_vel = Twist()
                cmd_vel.linear.x = no_vel_move
                cmd_vel.angular.z = no_vel_move
                self.vel_pub.publish(cmd_vel)

                # Remove the first element from the list to move to the next marker
                self.marker_list.pop(0)
                rospy.set_param('/marker_publisher/marker_list', self.marker_list)

            elif self.error < pixel_thr: # Check if the robot is aligned with the marker
                print("ROBOT AND MARKER ALIGNED!")
                cmd_vel = Twist()
                cmd_vel.linear.x = lin_vel_move
                cmd_vel.angular.z = no_vel_move
                self.vel_pub.publish(cmd_vel)

            else:
                # CONTROLLER for robot's alignment with markers
                cmd_vel = Twist()
                cmd_vel.linear.x = kp_d * self.error
                if cmd_vel.linear.x > ub_d:
                    cmd_vel.linear.x = ub_d

                if self.marker_center_x < width_camera:
                    cmd_vel.angular.z = kp_a * self.error
                    if cmd_vel.angular.z > ub_d:
                        cmd_vel.angular.z = ub_d
                else:
                    cmd_vel.angular.z = - kp_a * self.error
                    if cmd_vel.angular.z < - ub_d:
                        cmd_vel.angular.z = - ub_d

                self.vel_pub.publish(cmd_vel)

        else:
            # robot looking for the target marker
            print("ROBOT SEARCHING FOR MARKER...")
            cmd_vel = Twist()
            cmd_vel.linear.x = no_vel_move
            cmd_vel.angular.z = ang_vel_move
            self.vel_pub.publish(cmd_vel)


    def id_callback(self, data):
        # Callback function for marker ID
        self.marker_id = data.data

    def marker_center_callback(self, data):
        # Callback function for marker's center
        self.marker_center_x = data.x
        self.marker_center_y = data.y

    def pixel_callback(self, data):
        # Callback function for pixel side of the marker
        self.current_pixel_side = data.data


def main(args):
    '''Initializes and cleans up ROS node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

