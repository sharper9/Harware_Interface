#include <ros/ros.h>
#include <messages/PowerUsage.h>
#include <hw_interface_plugin_roboteq/Roboteq_Data.h>

ros::Time startTime;
ros::Publisher powerPub;

void rightRoboteqCallback(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr &msg)
{
    messages::PowerUsage powerMsg;
    float roboteqVolts = msg->analog_inputs[2] / 1000;
    float batteryVolts = msg->volts[1];
    float volts = (roboteqVolts / 40) * batteryVolts;
    powerMsg.time_step = startTime.toSec() - ros::Time::now().toSec();
    float joules = volts * powerMsg.time_step;
    powerMsg.usage = joules * 3600;
    powerPub.publish(powerMsg);
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "power_usage_node");
    startTime = ros::Time::now();
    
    ros::NodeHandle nh;
    ros::Subscriber rightRoboteqSub = nh.subscribe("/roboteq/drivemotorin/right", 1, &rightRoboteqCallback);
    powerPub = nh.advertise<messages::PowerUsage>("/energy_consumed", 1, false);

    ros::spin();
    return 0;
}
