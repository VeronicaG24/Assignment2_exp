#! /usr/bin/env python


from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import motion_plan.msg

def motion_client(pos_x, pos_y):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/reaching_goal', motion_plan.msg.PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = motion_plan.msg.PlanningGoal()
    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;
    goal.target_pose.pose.orientation.w = 0.0;

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = motion_client(3.0, 0.0)
        print("target reached")
        result = motion_client(3.0, 3.0)
        print("target reached")
        result = motion_client(0.0, 3.0)
        print("target reached")
        result = motion_client(0.0, 0.0)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
