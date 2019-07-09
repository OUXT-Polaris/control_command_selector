// Headers in ROS
#include <ros/ros.h>

// Headers in this package
#include <control_command_selector/control_command_selector.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_command_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ControlCommandSelector selector(nh,pnh);
    ros::spin();
    return 0;
}