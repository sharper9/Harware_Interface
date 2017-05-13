#include <ros/ros.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/nb2_3_to_i7_msg.h>
#include <armadillo>

#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

class IMU
{
private:
	ros::Subscriber subscriber_imu;
	ros::NodeHandle node;
	
  //netburner imu callback function
	void getIMUCallback(const messages::nb1_to_i7_msg::ConstPtr &msg)
	{
		//ROS_INFO("imu_callback 1 \n");
    this->p1 = msg->rate_p*3.1419527/180; //radians // TODO: Check this - quick fix to compile
    this->q1 = msg->rate_q*3.1419527/180; //radians // TODO: Check this
    this->r1 = msg->rate_r*3.1419527/180; //radians // TODO: Check this
    this->ax1 = msg->acc_x; //G's // TODO: Check this
    this->ay1 = msg->acc_y; //G's // TODO: Check this
    this->az1 = msg->acc_z; //G's // TODO: Check this
    this->imu_1_good = 1;//msg->pause_switch; //G's // TODO: Check this
		this->nb1_counter=msg->counter; //counter from nb
		++this->call_counter1; //increments each time function executed
		this->time1=msg->nb_clock; //time since object instantiated
	}


public:
	double start_time;
	double p, p1;
	double q, q1;
	double r, r1;
	double ax, ax1;
	double ay, ay1;
	double az, az1;

	double nb1_p;
	double nb1_q;
	double nb1_r;
	double nb1_ax;
	double nb1_ay;
	double nb1_az;

	double prev_time1;
	double dt;
	short int call_counter1;
	short int nb1_num_imus;
	short int nb1_counter;
	short int nb1_counter_prev;
	double time1;

	double p1_offset, q1_offset, r1_offset;

	double E_p1_offset, E_q1_offset, E_r1_offset;

	double mean_p1, mean_q1, mean_r1;

	double bias_thresh;
	bool good_bias1;
	short int new_imu1;
	bool imu_1_good;

	short int new_nb1, new_nb2, new_nbS;
	int nb1_missed_counter, nb2_missed_counter, nbS_missed_counter;
	int nb1_drive_counter, nb2_drive_counter, nbS_drive_counter;
	int nb1_diff_prev, nb2_diff_prev, nbS_diff_prev;
	bool nb1_current, nb2_current, nbS_current;
	bool nb1_good, nb2_good, nbS_good;
	bool nb1_good_prev, nb2_good_prev, nbS_good_prev;

	arma::mat p1_values;
	arma::mat q1_values;
	arma::mat r1_values;

	IMU();
	void collect_gyro1_data();

	void calculate_gyro1_offset();

	void set_gyro1_offset();

	void clear_gyro1_values();

	void clear_gyro_values();
	void filter_imu_values();
	void set_prev_counters();
	void determine_new_data();
};

#endif
