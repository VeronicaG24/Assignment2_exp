// Include the custom header file for the MyAction class, likely specific to your application.
#include "my_rosplan_interface/my_action.h"

// Include the Unix standard header for miscellaneous symbolic constants and types.
#include <unistd.h>

// Include the MoveBaseAction message definition from the move_base_msgs package.
// This is typically used for sending navigation goals to the move_base action server.
#include <move_base_msgs/MoveBaseAction.h>

// Include the SimpleActionClient from the actionlib package.
// This client is used to send goals to action servers and is a simpler interface than the full ActionClient.
#include <actionlib/client/simple_action_client.h> 

// Include the TerminalState from the actionlib package.
// This is used to get the state of an action (like succeeded, aborted, etc.) after it finishes.
#include <actionlib/client/terminal_state.h> 

// Include the Twist message definition from the geometry_msgs package.
#include <geometry_msgs/Twist.h>

// Include the Point message definition from the geometry_msgs package.

#include <geometry_msgs/Point.h>


namespace KCL_rosplan {

    // Constructor for the MyActionInterface class.
    MyActionInterface::MyActionInterface(ros::NodeHandle &nh) : nh_(nh) {
        
        // Initialize a publisher on the "/cmd_vel" topic to send Twist messages.
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Initialize a subscriber to the "/marker_point" topic.
        // The 'markerPointCallback' function of this class will handle the incoming messages.
        marker_point_subscriber_ = nh_.subscribe("/marker_point", 10, &MyActionInterface::markerPointCallback, this);
        
        // Initializing all the necessary variables.
        marker_center_x = 0.0;       // Initial value for the marker's center x-coordinate.
        width_camera = 320.0;        // Set the camera's width, used for calculating the error.
        flag = true;                 // A flag used to control the main loop in the action.
        error = 0.0;                 // Initialize the error to zero.
        pixel_thr = 18.0;            // Set the threshold for the pixel error.
    }

 
    // Callback function for the '/marker_point' topic subscriber.
    // It receives the position of a marker, particularly focusing on the x-coordinate.
    void MyActionInterface::markerPointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    
        // Update the marker_center_x variable with the x-coordinate from the message.
        marker_center_x = msg->x;

        ROS_INFO("Ricevuto punto marker: x = %f", marker_center_x);
    }

    // Declaration of the 'concreteCallback' method in the 'MyActionInterface' class.
    bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) { 
        
        // Rotate action implementation
        if (msg->name == "rotate") {         
            std::cout << "STIAMO FACENDO LA ROTAZIONE ";  
            
            // Continuously loop as long as the flag is true.
            while (flag) {
                // Calculate the absolute error between the center of the marker and the camera's width.
                error = std::abs(marker_center_x - width_camera);

                // Create a Twist message to control the robot's movement.
                geometry_msgs::Twist twist;
                twist.angular.z = 0.5; // Set a desired angular velocity.

                // Publish the twist message to the cmd_vel topic to rotate the robot.
                cmd_vel_pub_.publish(twist);
                
                // Check if the error is below the threshold (pixel_thr).
                if (error < pixel_thr) {
                    flag = false;  // Set the flag to false to exit the loop.

                    // Output a message indicating that the error is below the threshold.
                    std::cout << "Errore sotto la soglia, uscita dal ciclo di rotazione\n";

                    // Set the angular velocity to 0, stopping the robot's rotation.
                    twist.angular.z = 0.0;
                    
                    // Publish the twist message to stop the robot's rotation.
                    cmd_vel_pub_.publish(twist);
                }
            }

            // Updating the flag value in preparation for the next incoming rotate action
            flag = true;
        } 
        
        // goto_waypoint and come_back actions implementation
        else if (msg->name == "goto_waypoint" or msg->name == "come_back") {         
            
            if (msg->name == "come_back") {
                ROS_INFO("Going back to initial position...");
            } 
            else {
                ROS_INFO("Going to goal position...");
            }
          
            // Create a SimpleActionClient for the MoveBaseAction. It's used to send goals to the move_base server.
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

            // Instantiate a MoveBaseGoal object. This will be used to define the goal for the robot's movement.
            move_base_msgs::MoveBaseGoal goal;

            // Wait for the action server to start. This ensures that there is a connection to the server before sending goals.
            ac.waitForServer();
            
            // Set the frame of reference for the goal's target pose to "map".
            // This means the goal coordinates will be interpreted in the map's coordinate frame.
            goal.target_pose.header.frame_id = "map";

            // Set the orientation of the goal. Here, it sets the 'w' component of the quaternion to 1.0.
            // This represents a neutral orientation, meaning the robot will face forward relative to the map's frame.
            goal.target_pose.pose.orientation.w = 1.0;
           
            // Check if the third parameter of the message is "wp1"
            if (msg->parameters[2].value == "wp1") { 
                // If so, set the x and y coordinates of the goal's target position 
                goal.target_pose.pose.position.x = 5.5;
                goal.target_pose.pose.position.y = 2.5;
            }  
                     
            // Check if the third parameter of the message is "wp2"
            else if (msg->parameters[2].value == "wp2") {
                // If so, set the x and y coordinates of the goal's target position 
                goal.target_pose.pose.position.x = 7.0; 
                goal.target_pose.pose.position.y = -5.0;
            } 

            // Check if the third parameter of the message is "wp3"
            else if (msg->parameters[2].value == "wp3") {
                // If so, set the x and y coordinates of the goal's target position
                goal.target_pose.pose.position.x = -3.3;
                goal.target_pose.pose.position.y = -7.8;
            } 

            // Check if the third parameter of the message is "wp4"
            else if (msg->parameters[2].value == "wp4") {
                // If so, set the x and y coordinates of the goal's target position 
                goal.target_pose.pose.position.x = -7.0;
                goal.target_pose.pose.position.y = 1.0;
            } 

            // Check if the third parameter of the message is "wp0"
            else if (msg->parameters[2].value == "wp0") {
                // If so, set the x and y coordinates of the goal's target position 
                goal.target_pose.pose.position.x = 0.0;
                goal.target_pose.pose.position.y = 1.0;
            }  
            
            // Send the defined goal to the action server. This tells the move_base action server
            // to start working towards reaching the specified target pose.
            ac.sendGoal(goal);

            // Block the program until the action server reports a result. This means the code will
            // wait here until the robot has either reached the goal, failed to reach it, or the goal
            // has been preempted.
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

