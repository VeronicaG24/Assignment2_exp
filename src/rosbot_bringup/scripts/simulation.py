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
pixel_limit = 165  # limit for stopping the robot
width_camera = 320  # dimension of the camera
lin_vel_move = 0.2  # linear velocity
ang_vel_move = 0.5  # angular velocity
no_vel_move = 0.0  # stop velocity
pixel_thr = 18  # threshold in pixels for alignment
yaw_thr = math.pi / 90  # +/- 2 degree allowed for yaw

kp_d = 0.2  # control distance gain
kp_a = 3.0  # control angular gain
ub_d = 0.3  # upper bound distance
ub_cr = 0.4  # upper bound camera rotation
ub_br = 0.5  # upper bound base rotation


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

        # Subscriber for robot/camera pose and orientation
        self.subscriber_pose = rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_callback, queue_size=1)

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

        self.orientation_robot = 0.0  # orientation of the robot
        self.orientation_camera = 0.0  # orientation of the camera

        self.yaw_robot = 0.0  # yaw of the robot
        self.yaw_camera = 0.0  # yaw of the camera

        self.alligned = False  # used to align the robot with the center of the marker

    def pose_callback(self, data):
        # Ensure that the 'pose' list is not empty
        if len(data.pose) >= 10:
            # Extract orientation information from the robot
            self.orientation_robot = (
                data.pose[9].orientation.x, data.pose[9].orientation.y, data.pose[9].orientation.z, data.pose[9].orientation.w)
            quaternion_robot = (
                self.orientation_robot[0],
                self.orientation_robot[1],
                self.orientation_robot[2],
                self.orientation_robot[3])
            # compute Euler transformation for the robot
            euler_robot = transformations.euler_from_quaternion(quaternion_robot)
            # get yaw of the robot
            self.yaw_robot = euler_robot[2]

            # Extract orientation information from the camera
            self.orientation_camera = (
                data.pose[10].orientation.x, data.pose[10].orientation.y, data.pose[10].orientation.z, data.pose[10].orientation.w)
            quaternion_camera = (
                self.orientation_camera[0],
                self.orientation_camera[1],
                self.orientation_camera[2],
                self.orientation_camera[3])
            # compute Euler transformation for the camera
            euler_camera = transformations.euler_from_quaternion(quaternion_camera)
            # get yaw of the camera
            self.yaw_camera = euler_camera[2]

    def move_callback(self, ros_data):
        # Check if the marker list is empty
        if not self.marker_list:
            # Send a message to close marker_publisher on the topic where we write the marker number
            rospy.signal_shutdown("All markers reached")
            return

        # operation mode: camera moving
        if self.marker_id == self.marker_list[0]:

            print("MARKER FOUND: " + str(self.marker_id))

            self.yaw_error = self.normalize_angle(self.yaw_camera - self.yaw_robot) # Error between robot base's orientation and camera's orientation

            self.error = abs(self.marker_center_x - width_camera) # Error between the marker's center and the camera's center

            if abs(self.yaw_error) <= yaw_thr or self.alligned == True:
                # robot aligned with the center of the marker

                vel_camera = Float64()
                vel_camera.data = 0.0
                self.joint_state_pub.publish(vel_camera)

                self.alligned = True

                if self.current_pixel_side > pixel_limit and self.alligned == True:
                    # stop the robot when the marker is reached
                    print("MARKER REACHED: " + str(self.marker_id))
                    cmd_vel = Twist()
                    cmd_vel.linear.x = no_vel_move
                    cmd_vel.angular.z = no_vel_move
                    self.vel_pub.publish(cmd_vel)

                    # Remove the first element from the list to move to the next marker
                    self.marker_list.pop(0)
                    rospy.set_param('/marker_publisher/marker_list', self.marker_list)
                    self.alligned = False

                elif self.error < pixel_thr and self.alligned == True:
                    # robot aligned with the center of the marker
                    print("robot and marker ALIGNED!")

                    cmd_vel = Twist()
                    cmd_vel.linear.x = lin_vel_move
                    cmd_vel.angular.z = no_vel_move
                    self.vel_pub.publish(cmd_vel)

                else:
                    if self.alligned:
                        # CONTROLLER for robot's alignment with marker
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
                # allign the robot with camera
                cmd_vel = Twist()
                cmd_vel.linear.x = no_vel_move
                cmd_vel.angular.z = kp_a * abs(self.yaw_error)

                if cmd_vel.angular.z > ub_br:
                    cmd_vel.angular.z = ub_br

                self.vel_pub.publish(cmd_vel)

                vel_camera = Float64()
                vel_camera.data = - kp_a * abs(self.yaw_error) + 0.1 # + 0.1 is due to the fact that the camera's velocity has to be slower than robot base's velocity
                if vel_camera.data < - ub_cr:
                    vel_camera.data = - ub_cr
                self.joint_state_pub.publish(vel_camera)

        else:
            # camera searching for the target marker
            vel_camera = Float64()
            print("CAMERA SEARCHING FOR MARKER...")
            vel_camera.data = ang_vel_move
            self.joint_state_pub.publish(vel_camera)

    def normalize_angle(self, angle):
        # function for normalizing angles
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / math.fabs(angle)
        return angle

    def id_callback(self, data):
        # Callback function for marker ID
        self.marker_id = data.data

    def marker_center_callback(self, data):
        # Callback function for marker center
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

