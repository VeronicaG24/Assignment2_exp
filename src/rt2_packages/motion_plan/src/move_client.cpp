#include <ros/ros.h>
#include <motion_plan/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float64.h>
#include <unistd.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("/reaching_goal", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  double pos_x[4] = {3.0, 3.0, 0.0, 0.0};
  double pos_y[4] = {0.0, 3.0, 3.0, 0.0};

  motion_plan::PlanningGoal goal;
  
  
  for(int i=0;i<4;i++){
  //we'll send a goal to move the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pos_x[i];
  goal.target_pose.pose.position.y = pos_y[i];
  goal.target_pose.pose.orientation.w = 0.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, target reached!");
  else
    ROS_INFO("The base failed to reach the target for some reason");
  
  } 


  return 0;
}
