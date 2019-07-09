// Headers in this package
#include <control_command_selector/control_command_selector.h>

ControlCommandSelector::ControlCommandSelector(ros::NodeHandle nh,ros::NodeHandle pnh) : event_client_(nh,pnh,"control_state_machine_node")
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("manual_command_topic", manual_command_topic_, "/manual_command");
    pnh_.param<std::string>("control_command_topic", control_command_topic_, "/control_command");
    event_client_.run();
    manual_command_sub_ = nh_.subscribe(manual_command_topic_,1,&ControlCommandSelector::manualComandCallback,this);
    control_command_sub_ = nh_.subscribe(control_command_topic_,1,&ControlCommandSelector::controllerComandCallback,this);
}

ControlCommandSelector::~ControlCommandSelector()
{

}

void ControlCommandSelector::manualComandCallback(const usv_control_msgs::AzimuthThrusterCatamaranDriveStamped::ConstPtr msg)
{

}

void ControlCommandSelector::controllerComandCallback(const usv_control_msgs::AzimuthThrusterCatamaranDriveStamped::ConstPtr msg)
{

}