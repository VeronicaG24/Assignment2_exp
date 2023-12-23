#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
// add the following three include
#include <actionlib/client/simple_action_client.h> 
#include <actionlib/client/terminal_state.h> 
#include <motion_plan/PlanningAction.h>

#include <geometry_msgs/Twist.h>


namespace KCL_rosplan {

MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
    // here the initialization
}

    bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) { // concreteCallback deve avere lo stesso nome in my_action.h
        // here the implementation of the action
        //std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
        //std::cout << "Received message: " << *msg << std::endl;
        if (msg->name == "rotate") {         
            std::cout << "STIAMO FACENDO LA ROTAZIONE ";  
        } 
        
        else if (msg->name == "goto_waypoint") {         
            std::cout << "GOING... ";      
        }
        sleep(5);
        
        
        

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

