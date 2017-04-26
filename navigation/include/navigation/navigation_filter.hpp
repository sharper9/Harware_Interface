#ifndef NAVIGATION_FILTER_H
#define NAVIGATION_FILTER_H
#include <ros/ros.h>
#include <navigation/encoders_class.hpp>
#include <navigation/imu_class.hpp>
#include <navigation/filter_class.hpp>
#include <messages/ExecInfo.h>
#include <messages/MissionPlanningInfo.h>
#include <messages/NavFilterOut.h>

//#include <messages/NavFilterControl.h> //added for new User Interface -Matt G.

#define DEG_2_RAD (PI/180.0)
#define RAD_2_DEG (180.0/PI)

#define REG_WHEEL_RADIUS 0.231775
#define REG_WHEEL_COUNTS_PER_REV_FRONT 2451
#define REG_WHEEL_COUNTS_PER_REV_BACK 1363

#define COMP_WHEEL_RADIUS 0.200025
#define COMP_WHEEL_COUNTS_PER_REV_FRONT 2390
#define COMP_WHEEL_COUNTS_PER_REV_BACK 1358

class NavigationFilter
{
	public:
		// Methods
		NavigationFilter();
		void update_time();
    // TODO: Check this, User_Input_Nav_Act from cataglyphis hsm - commented out for compiling
//		void waiting(User_Input_Nav_Act user_input_nav_act);
//		void forklift_drive(User_Input_Nav_Act user_input_nav_act);
//    void run(User_Input_Nav_Act user_input_nav_act);
    void run();
	    //added for new User Interface -Matt G.
//	    bool navFilterControlServiceCallback(messages::NavFilterControl::Request request, messages::NavFilterControl::Response response);
		// Members
		ros::NodeHandle nh;

	    ros::ServiceServer nav_control_server;
      // TODO: Check this - commented out to compile
//	    messages::NavFilterControl::Request latest_nav_control_request;

		ros::Subscriber sub_exec;
		bool pause_switch;
		bool stopFlag;
		bool turnFlag;

		Encoders encoders;
		IMU imu;
		Filter filter;
    Filter init_filter; // TODO: check this - added for lines 68-70 in navigation_filter_node.cpp

		double current_time;
		double stop_time;
		double dt = 0;

		const double PI = 3.14159265;
		const double G = 9.80665;
		int calibrate_counter = 0;
		const double calibrate_time = 10.0;
		bool prev_stopped = true;
		bool collecting_accelerometer_data = false;
		bool collected_gyro_data = false;
		bool first_pass = true;
		bool stop_request = false;
		int nav_status_output = 0;

	private:
		void getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg);

	    //added for new User Interface -Matt G.
//	    bool navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response);
};

#endif // NAVIGATION_FILTER_H
