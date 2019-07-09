// Headers in this package
#include <control_command_selector/control_command_selector.h>

ControlCommandSelector::ControlCommandSelector(ros::NodeHandle nh,ros::NodeHandle pnh) : event_client_(nh,pnh,"control_state_machine_node")
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("manual_command_topic", manual_command_topic_, "/manual_command");
    pnh_.param<std::string>("control_command_topic", control_command_topic_, "/control_command");
    pnh_.param<std::string>("current_command_topic", current_command_topic_, "/current_command");
    
    current_command_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(current_command_topic_,1);
    event_client_.registerCallback(std::bind(&ControlCommandSelector::publishZeroCommandAsCurrentCommand, this),
        "ControlCommandSelector::publishZeroCommandAsCurrentCommand");
    event_client_.registerCallback(std::bind(&ControlCommandSelector::publishControlCommandAsCurrentCommand, this),
        "ControlCommandSelector::publishControlCommandAsCurrentCommand");
    event_client_.registerCallback(std::bind(&ControlCommandSelector::publishManualCommandAsCurrentCommand, this),
        "ControlCommandSelector::publishManualCommandAsCurrentCommand");
    event_client_.run();
    manual_command_sub_ = nh_.subscribe(manual_command_topic_,1,&ControlCommandSelector::manualComandCallback,this);
    control_command_sub_ = nh_.subscribe(control_command_topic_,1,&ControlCommandSelector::controllerComandCallback,this);
}

ControlCommandSelector::~ControlCommandSelector()
{

}

void ControlCommandSelector::manualComandCallback(const usv_control_msgs::AzimuthThrusterCatamaranDriveStamped::ConstPtr msg)
{
    manual_command_ = *msg;
    return;
}

void ControlCommandSelector::controllerComandCallback(const usv_control_msgs::AzimuthThrusterCatamaranDriveStamped::ConstPtr msg)
{
    control_command_ = *msg;
    return;
}

boost::optional<rostate_machine::Event> ControlCommandSelector::publishZeroCommandAsCurrentCommand()
{
    usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    current_command_pub_.publish(msg);
    return boost::none;
}

boost::optional<rostate_machine::Event> ControlCommandSelector::publishControlCommandAsCurrentCommand()
{
    if(control_command_)
    {
        current_command_pub_.publish(*control_command_);
    }
    else
    {
        usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        current_command_pub_.publish(msg);
    }
    return boost::none;
}

boost::optional<rostate_machine::Event> ControlCommandSelector::publishManualCommandAsCurrentCommand()
{
    if(manual_command_)
    {
        current_command_pub_.publish(*manual_command_);
    }
    else
    {
        usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        current_command_pub_.publish(msg);
    }
    return boost::none;
}