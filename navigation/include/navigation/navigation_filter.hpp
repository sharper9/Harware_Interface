#ifndef NAVIGATION_FILTER_H
#define NAVIGATION_FILTER_H
#include <ros/ros.h>
#include <navigation/encoders_class.hpp>
#include <navigation/imu_class.hpp>
#include <navigation/filter_class.hpp>
#include <messages/ExecInfo.h>
#include <messages/MissionPlanningInfo.h>
#include <messages/NavFilterOut.h>
#include <td_navigation/Running_Half_Pose.h>
#include <td_navigation/Localize.h>

//#include <messages/NavFilterControl.h> //added for new User Interface -Matt G.

#define DEG_2_RAD (PI/180.0)
#define RAD_2_DEG (180.0/PI)

#define REG_WHEEL_RADIUS 0.23733
#define REG_WHEEL_COUNTS_PER_REV_FRONT_RIGHT 1866
#define REG_WHEEL_COUNTS_PER_REV_FRONT_LEFT 1779
#define REG_WHEEL_COUNTS_PER_REV_BACK_RIGHT 1312
#define REG_WHEEL_COUNTS_PER_REV_BACK_LEFT 1562

#define COMP_WHEEL_RADIUS 0.200025
#define COMP_WHEEL_COUNTS_PER_REV_FRONT_RIGHT 2507.8335
#define COMP_WHEEL_COUNTS_PER_REV_FRONT_LEFT 2574.5
#define COMP_WHEEL_COUNTS_PER_REV_BACK_RIGHT 1599.0333
#define COMP_WHEEL_COUNTS_PER_REV_BACK_LEFT 1648.2283

#define USE_COMP_WHEELS

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

    ros::ServiceClient ranging_radio_client;
	    ros::ServiceServer nav_control_server;
      // TODO: Check this - commented out to compile
//	    messages::NavFilterControl::Request latest_nav_control_request;

		ros::Subscriber sub_exec;
                ros::Subscriber sub_mission_planning;
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

    const int NUMBER_OF_DATA_POINTS_BIAS_REMOVAL = 250;
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
        const float initX = 1.0; // m
        const float initY = 0.0; // m
        const float initHeading = 0.0; // rad


        ros::Subscriber sub_rr_half_pose;
        bool rr_new_half_pose=false;
        td_navigation::Running_Half_Pose rr_new_pose;
        float rr_position_update_moving_tolerence = 2.0; //meters
        
        bool rr_pose_update_in_progress = false;
        bool rr_full_pose_failed=false;

        bool rr_initial_pose_found = false;
        bool perform_rr_heading_update = false;
        const float rr_heading_update_tolerance = 15.0*DEG_2_RAD;
        const float rr_error_heading_update_tolerance = 18*DEG_2_RAD;
        const float rr_stopped_position_update_tolerence = 2.0; //meters
        bool rr_found_full_pose = false;
        int rr_full_pose_failed_counter = 0;
        const int rr_full_pose_failed_max_count = 3;

        ros::Publisher nav_pub;
	    messages::NavFilterOut msg_NavFilterOut;
	    
	    void packInfoMsgAndPub();


	private:
		void getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg);
                void getMissionPlanningInfoCallback(const messages::MissionPlanningInfo::ConstPtr &msg);
                void getRRHalfPose(const td_navigation::Running_Half_Pose::ConstPtr &msg);

	    //added for new User Interface -Matt G.
//	    bool navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response);
};

#endif // NAVIGATION_FILTER_H
