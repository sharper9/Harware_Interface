#include <ros/ros.h>
#include <navigation/navigation_filter.hpp>
//#include <hsm/user_input_nav_act_class.h> TODO: Check this - do we need to add hsm?
#include <messages/NavFilterOut.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigation_filter_node");
	ROS_INFO("navigation_filter_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	NavigationFilter navigationfilter;

	while(ros::ok())
	{
		//update navigation filtert time
		navigationfilter.filter.counter++;
		navigationfilter.update_time(); //updates dt and current_time

		//filter imu data
		navigationfilter.imu.determine_new_data();
		navigationfilter.imu.filter_imu_values();
		navigationfilter.imu.set_prev_counters();

		//calculate distance from encoders
		navigationfilter.encoders.adjustEncoderWrapError();
		navigationfilter.encoders.calculateWheelDistancesFromEncoders();
		navigationfilter.encoders.calculateDeltaDistance4Wheels(0, 0); //turnFlag, stopFlag

		//execute navigation filter
		navigationfilter.run();

        navigationfilter.packInfoMsgAndPub();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
