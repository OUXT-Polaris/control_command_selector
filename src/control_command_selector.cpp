// Headers in this package
#include <control_command_selector/control_command_selector.h>

ControlCommandSelector::ControlCommandSelector(ros::NodeHandle nh,ros::NodeHandle pnh) : event_client_(nh,pnh,"control_state_machine_node")
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("manual_command_topic", manual_command_topic_, "/manual_command");
    pnh_.param<std::string>("control_command_topic", control_command_topic_, "/control_command");
    pnh_.param<std::string>("motor_command_topic", motor_command_topic_, "/motor_command");
    
    motor_command_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(motor_command_topic_,1);
    event_client_.registerCallback(std::bind(&ControlCommandSelector::publishZeroCommandAsMotorCommand, this),
        "ControlCommandSelector::publishZeroCommandAsMotorCommand");
    event_client_.registerCallback(std::bind(&ControlCommandSelector::publishControlCommandAsMotorCommand, this),
        "ControlCommandSelector::publishControlCommandAsMotorCommand");
    event_client_.registerCallback(std::bind(&ControlCommandSelector::publishManualCommandAsMotorCommand, this),
        "ControlCommandSelector::publishManualCommandAsMotorCommand");
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

boost::optional<rostate_machine::Event> ControlCommandSelector::publishZeroCommandAsMotorCommand()
{
    usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    motor_command_pub_.publish(msg);
    return boost::none;
}

boost::optional<rostate_machine::Event> ControlCommandSelector::publishControlCommandAsMotorCommand()
{
    if(control_command_)
    {
        motor_command_pub_.publish(*control_command_);
    }
    else
    {
        usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        motor_command_pub_.publish(msg);
    }
    return boost::none;
}

boost::optional<rostate_machine::Event> ControlCommandSelector::publishManualCommandAsMotorCommand()
{
    if(manual_command_)
    {
        motor_command_pub_.publish(*manual_command_);
    }
    else
    {
        usv_control_msgs::AzimuthThrusterCatamaranDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        motor_command_pub_.publish(msg);
    }
    return boost::none;
}