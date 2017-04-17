#include <ros/ros.h>
#include <messages/encoder_data.h>

#ifndef ENCODERS_CLASS_HPP
#define ENCODERS_CLASS_HPP

class Encoders
{
private:
	ros::Subscriber subscriber_encoder_left;
	ros::Subscriber subscriber_encoder_right;
	ros::NodeHandle node;

	// TODO: This callback needs to be updated for new roboteq feedback format
	void getEncoderLeftCallback(const messages::encoder_data::ConstPtr &msg)
	{
		this->fl_prev = this->fl;
		this->bl_prev = this->bl;
		this->fl = msg->motor_1_encoder_count;
		this->ml = msg->motor_2_encoder_count;
		this->bl = msg->motor_3_encoder_count;
		this->fl_diff = this->fl - this->fl_prev;
		this->ml_diff = this->ml - this->ml_prev;
		this->bl_diff = this->bl - this->bl_prev;
		this->counter_left=this->counter_left+1;
	}

	// TODO: This callback needs to be updated for new roboteq feedback format
	void getEncoderRightCallback(const messages::encoder_data::ConstPtr &msg)
	{
		this->fr_prev = this->fr;
		this->mr_prev = this->mr;
		this->br_prev = this->br;
		this->fr = msg->motor_1_encoder_count;
		this->mr = msg->motor_2_encoder_count;
		this->br = msg->motor_3_encoder_count;
		this->fr_diff = this->fr - this->fr_prev;
		this->mr_diff = this->mr - this->mr_prev;
		this->br_diff = this->br - this->br_prev;
		this->counter_right=this->counter_right+1;
	}
public:
	short int counter_left;
	short int counter_left_prev;
	short int counter_right;
	short int counter_right_prev;
	long int fl;
	long int fr;
	long int bl;
	long int br;
	long int fl_prev;
	long int fr_prev;
	long int bl_prev;
	long int br_prev;
	long int fl_diff;
	long int fr_diff;
	long int bl_diff;
	long int br_diff;
	double fl_dist;
	double fr_dist;
	double bl_dist;
	double br_dist;
	long int encoder_max_count;
	long int impossible_encoder_diff;
	long int spike_diff;
	float wheel_radius;
	double delta_distance;
	int counts_per_revolution;
	Encoders();
	void set_wheel_radius(double set_radius);
	void set_counts_per_revolution(double set_counts);
	void adjustEncoderWrapError();
	void calculateWheelDistancesFromEncoders();
	void calculateDeltaDistance4Wheels(int turnFlag, int stopFlag);
};

#endif
