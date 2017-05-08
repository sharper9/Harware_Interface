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

	ros::Publisher pub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
	messages::NavFilterOut msg_NavFilterOut;

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

		//add navigation information to message
    msg_NavFilterOut.p1 = navigationfilter.imu.p1;
		msg_NavFilterOut.q1 = navigationfilter.imu.q1;
		msg_NavFilterOut.r1 = navigationfilter.imu.r1;
		msg_NavFilterOut.p1_offset = navigationfilter.imu.p1_offset;
		msg_NavFilterOut.q1_offset = navigationfilter.imu.q1_offset;
		msg_NavFilterOut.r1_offset = navigationfilter.imu.r1_offset;
		msg_NavFilterOut.roll_rate = navigationfilter.imu.p;
		msg_NavFilterOut.pitch_rate = navigationfilter.imu.q;
		msg_NavFilterOut.yaw_rate = navigationfilter.imu.r;
		msg_NavFilterOut.ax = navigationfilter.imu.ax1;
		msg_NavFilterOut.ay = navigationfilter.imu.ay1;
		msg_NavFilterOut.az = navigationfilter.imu.az1;
		msg_NavFilterOut.dt = navigationfilter.dt;
		msg_NavFilterOut.x_position = navigationfilter.filter.x;
		msg_NavFilterOut.y_position = navigationfilter.filter.y;
		msg_NavFilterOut.roll = navigationfilter.filter.phi*180/3.1415927; 
		msg_NavFilterOut.pitch = navigationfilter.filter.theta*180/3.1415927;
		msg_NavFilterOut.heading = navigationfilter.filter.psi*180/3.1415927;
		msg_NavFilterOut.human_heading = fmod(navigationfilter.filter.psi*180/3.1415927,360);
		msg_NavFilterOut.bearing = atan2(navigationfilter.filter.y,navigationfilter.filter.x)*180/3.1415927;
		msg_NavFilterOut.velocity = navigationfilter.encoders.delta_distance/navigationfilter.dt;
		msg_NavFilterOut.delta_distance = navigationfilter.encoders.delta_distance;
		msg_NavFilterOut.roll_init = navigationfilter.init_filter.phi*180.0/navigationfilter.PI;
		msg_NavFilterOut.pitch_init = navigationfilter.init_filter.theta*180.0/navigationfilter.PI;
		msg_NavFilterOut.heading_init = navigationfilter.init_filter.psi*180.0/navigationfilter.PI;
		msg_NavFilterOut.counter=navigationfilter.filter.counter;
		msg_NavFilterOut.nav_status = navigationfilter.nav_status_output;
		msg_NavFilterOut.imu_call_counter = navigationfilter.imu.call_counter1;

		//publish navigation message
		pub.publish(msg_NavFilterOut);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
