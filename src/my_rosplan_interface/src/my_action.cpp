#include "my_rosplan_interface/my_action.h"
#include <unistd.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> 
#include <actionlib/client/terminal_state.h> 
#include <motion_plan/PlanningAction.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>




namespace KCL_rosplan {

    MyActionInterface::MyActionInterface(ros::NodeHandle &nh) : nh_(nh) {
        // Creazione del publisher cmd_vel
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        id_subscriber_ = nh_.subscribe("/id_publisher", 10, &MyActionInterface::idCallback, this);
        marker_point_subscriber_ = nh_.subscribe("/marker_point", 10, &MyActionInterface::markerPointCallback, this);
        
        marker_center_x = 0.0;
        marker_id = 0;
        width_camera = 320.0;
        flag = true;
        error = 0.0;
        pixel_thr = 18.0;

    }
    
    void MyActionInterface::idCallback(const std_msgs::Int32::ConstPtr& msg) {
        marker_id = msg->data;
       // ROS_INFO("Ricevuto ID marker: %d", marker_id);
    }
    
    void MyActionInterface::markerPointCallback(const geometry_msgs::Point::ConstPtr& msg) {
        marker_center_x = msg->x;
        ROS_INFO("Ricevuto punto marker: x = %f", marker_center_x);
    }
    

    bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) { // concreteCallback deve avere lo stesso nome in my_action.h
        // here the implementation of the action
        //std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
        //std::cout << "Received message: " << *msg << std::endl;
        if (msg->name == "rotate") {         
            std::cout << "STIAMO FACENDO LA ROTAZIONE ";  
            
            
            while (flag) {
          
                error = std::abs(marker_center_x - width_camera);
                geometry_msgs::Twist twist;
                twist.angular.z = 0.4; // Imposta la velocità angolare desiderata
                cmd_vel_pub_.publish(twist);
                
                 // Controlla se l'errore è sotto la soglia
                if (error < pixel_thr) {
                    flag = false;  // Imposta la flag su false per uscire dal ciclo
                    std::cout << "Errore sotto la soglia, uscita dal ciclo di rotazione\n";
                    twist.angular.z = 0.0; // Imposta la velocità angolare desiderata
                    cmd_vel_pub_.publish(twist);
                }
            }
            
            flag = true;
            sleep(0.5);

        } 
        
        else if (msg->name == "goto_waypoint" or msg->name == "come_back") {         
            
            if (msg->name == "come_back") {
                std::cout << "Going back to initial position... "; 
            }
            else {
                std::cout << "Going to goal position... ";
            }
            
       
            
            
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); 
            move_base_msgs::MoveBaseGoal goal;
            ac.waitForServer();
            
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.pose.orientation.w = 1.0;
            
            if(msg->parameters[2].value == "wp1"){ 
                goal.target_pose.pose.position.x = 5.5;//5.5;
                goal.target_pose.pose.position.y = 1.5;//1.5;
                
            }  
             
            else if (msg->parameters[2].value == "wp2"){
                 goal.target_pose.pose.position.x = 6.0; //6.1;
                 goal.target_pose.pose.position.y = -5.0;//-5.8;
                 
            } 
            
            else if (msg->parameters[2].value == "wp3"){
                 goal.target_pose.pose.position.x = -3.0;
                 goal.target_pose.pose.position.y = -7.0;
                
            } 
            
            else if (msg->parameters[2].value == "wp4"){
                 goal.target_pose.pose.position.x = -7.0;
                 goal.target_pose.pose.position.y = 2.0;
                 
            } 
            
            else if (msg->parameters[2].value == "wp0"){
                 goal.target_pose.pose.position.x = 0.0;
                 goal.target_pose.pose.position.y = 1.0;
                 
            } 
            
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

