#include <ros/ros.h>
#include <messages/PowerUsage.h>
#include <hw_interface_plugin_roboteq/Roboteq_Data.h>

ros::Time startTime;
ros::Publisher powerPub;

void rightRoboteqCallback(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr &msg)
{
    messages::PowerUsage powerMsg;
    int16_t roboteqVolts = boost::lexical_cast<int16_t>(msg->analog_inputs[2]) / 1000;
    int16_t batteryVolts = boost::lexical_cast<int16_t>(msg->volts[1]) / 10;
    int16_t volts = (roboteqVolts / 40) * batteryVolts;
    powerMsg.time_step = startTime.toSec() - ros::Time::now().toSec();
    int16_t joules = volts * powerMsg.time_step;
    powerMsg.usage = joules * 3600;
    powerPub.publish(powerMsg);
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "power_usage_node");
    ros::NodeHandle nh;
    startTime = ros::Time::now();
    
    ros::Subscriber rightRoboteqSub = nh.subscribe("/roboteq/drivemotorin/right", 1, &rightRoboteqCallback);
    powerPub = nh.advertise<messages::PowerUsage>("/energy_consumed", 1, false);

    ros::spin();
    return 0;
}
