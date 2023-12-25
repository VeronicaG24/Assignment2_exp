#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
// add the following three include
#include <actionlib/client/simple_action_client.h> 
#include <actionlib/client/terminal_state.h> 
#include <motion_plan/PlanningAction.h>

#include <geometry_msgs/Twist.h>


namespace KCL_rosplan {

    MyActionInterface::MyActionInterface(ros::NodeHandle &nh) : nh_(nh) {
        // Creazione del publisher cmd_vel
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) { // concreteCallback deve avere lo stesso nome in my_action.h
        // here the implementation of the action
        //std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
        //std::cout << "Received message: " << *msg << std::endl;
        if (msg->name == "rotate") {         
            std::cout << "STIAMO FACENDO LA ROTAZIONE ";  
            
            geometry_msgs::Twist twist;
            twist.angular.z = 0.5; // Imposta la velocità angolare desiderata
            cmd_vel_pub_.publish(twist);
            
            sleep(10);

        } 
        
        else if (msg->name == "goto_waypoint") {         
            std::cout << "GOING... ";  
            
            
            actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true); 
            motion_plan::PlanningGoal goal;
            ac.waitForServer();
            
            
            
            if(msg->parameters[2].value == "wp1"){ 
                goal.target_pose.pose.position.x = 5.5;
                goal.target_pose.pose.position.y = 1.5;
                goal.target_pose.pose.orientation.w = 0.0;
            }  
             
            else if (msg->parameters[2].value == "wp2"){
                 goal.target_pose.pose.position.x = 6.1;
                 goal.target_pose.pose.position.y = -5.8;
                 goal.target_pose.pose.orientation.w = 0.0;
            } 
            
            else if (msg->parameters[2].value == "wp3"){
                 goal.target_pose.pose.position.x = -3.0;
                 goal.target_pose.pose.position.y = -7.0;
                 goal.target_pose.pose.orientation.w = 0.0;
            } 
            
            else if (msg->parameters[2].value == "wp4"){
                 goal.target_pose.pose.position.x = -7.0;
                 goal.target_pose.pose.position.y = 2.0;
                 goal.target_pose.pose.orientation.w = 0.0;
            } 
            
            ac.sendGoal(goal); 
            ac.waitForResult();


        }
        
        else if (msg->name == "come_back") {         
            std::cout << "Going back to initial position... "; 
            
            actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true); 
            motion_plan::PlanningGoal goal;
            ac.waitForServer();
            
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y = 1.0;
            goal.target_pose.pose.orientation.w = 0.0; 
            
            ac.sendGoal(goal); 
            ac.waitForResult();    
        }
        
        
        
        

        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }

} // namespace KCL_rosplan

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::MyActionInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}

