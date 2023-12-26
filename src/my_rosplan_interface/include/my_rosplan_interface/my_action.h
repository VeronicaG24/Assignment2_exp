#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>



namespace KCL_rosplan {

class MyActionInterface : public RPActionInterface {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber id_subscriber_;
    ros::Subscriber marker_point_subscriber_;
    
    double width_camera;
    double marker_center_x;
    int marker_id;
    bool flag;
    double error;
    double pixel_thr;
   

public:
    /* constructor */
    MyActionInterface(ros::NodeHandle &nh);

    /* listen to and process action_dispatch topic */
    bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    void idCallback(const std_msgs::Int32::ConstPtr& msg);
    void markerPointCallback(const geometry_msgs::Point::ConstPtr& msg);
};

} // namespace KCL_rosplan

