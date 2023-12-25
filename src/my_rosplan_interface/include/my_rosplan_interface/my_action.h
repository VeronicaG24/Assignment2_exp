#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <geometry_msgs/Twist.h>

namespace KCL_rosplan {

class MyActionInterface : public RPActionInterface {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

public:
    /* constructor */
    MyActionInterface(ros::NodeHandle &nh);

    /* listen to and process action_dispatch topic */
    bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

} // namespace KCL_rosplan

