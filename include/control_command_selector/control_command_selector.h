#ifndef CONTROL_COMMAND_SELECTOR_CONTROL_COMMAND_SELECTOR_H_INCLUDED
#define CONTROL_COMMAND_SELECTOR_CONTROL_COMMAND_SELECTOR_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>
#include <rostate_machine/event_client.h>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>

class ControlCommandSelector
{
public:
    ControlCommandSelector(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~ControlCommandSelector();
    void manualComandCallback(const usv_control_msgs::AzimuthThrusterCatamaranDriveStamped::ConstPtr msg);
    void controllerComandCallback(const usv_control_msgs::AzimuthThrusterCatamaranDriveStamped::ConstPtr msg);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string manual_command_topic_;
    std::string control_command_topic_;
    std::string current_command_topic_;
    ros::Subscriber manual_command_sub_;
    ros::Subscriber control_command_sub_;
    ros::Publisher current_command_pub_;
    boost::optional<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped> manual_command_;
    boost::optional<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped> control_command_;
    rostate_machine::EventClient event_client_;
    boost::optional<rostate_machine::Event> publishZeroCommandAsCurrentCommand();
    boost::optional<rostate_machine::Event> publishControlCommandAsCurrentCommand();
    boost::optional<rostate_machine::Event> publishManualCommandAsCurrentCommand();
};

#endif  //CONTROL_COMMAND_SELECTOR_CONTROL_COMMAND_SELECTOR_H_INCLUDED