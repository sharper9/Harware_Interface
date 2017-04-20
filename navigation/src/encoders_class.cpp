#include <navigation/encoders_class.hpp>

Encoders::Encoders()
{
	counter_left=0;
	counter_right=0;
	counter_left_prev=0;
	counter_right_prev=0;
	fl=0;
	fr=0;
	bl=0;
	br=0;
	fl_prev=0;
	fr_prev=0;
	bl_prev=0;
	br_prev=0;
	fl_diff=0;
	fr_diff=0;
	bl_diff=0;
	br_diff=0;
    encoder_max_count=2147483648;
  impossible_encoder_diff=10000; // TODO: check this
	spike_diff = 1000;
	wheel_radius=0;
	counts_per_revolution=1;
    subscriber_encoder_left = node.subscribe("roboteq/drivemotorin/left", 1, &Encoders::getEncoderLeftCallback,this); // TODO: update topic name
    subscriber_encoder_right = node.subscribe("roboteq/drivemotorin/right", 1, &Encoders::getEncoderRightCallback,this); // TODO: update topic name
}

void Encoders::set_wheel_radius(double set_radius)
{
	wheel_radius=set_radius;
}

void Encoders::set_counts_per_revolution(double set_counts)
{
	counts_per_revolution=set_counts;
}
// TODO
void Encoders::adjustEncoderWrapError()
{
	if(abs(fl_diff)>impossible_encoder_diff)
	{
		if (fl_prev > encoder_max_count/2) fl_diff = (encoder_max_count-fl_prev)+fl;
		else fl_diff = -fl_prev-(encoder_max_count-fl);
		if(abs(fl_diff)>spike_diff) fl_diff = 0;
	}

	if(abs(fr_diff)>impossible_encoder_diff)
	{
		if (fr_prev > encoder_max_count/2) fr_diff = (encoder_max_count-fr_prev)+fr;
		else fr_diff = -fr_prev-(encoder_max_count-fr);
		if(abs(fr_diff)>spike_diff) fr_diff = 0;
	}

	if(abs(bl_diff)>impossible_encoder_diff)
	{
		if (bl_prev > encoder_max_count/2) bl_diff = (encoder_max_count-bl_prev)+bl;
		else bl_diff = -bl_prev-(encoder_max_count-bl);
		if(abs(bl_diff)>spike_diff) bl_diff = 0;
	}

	if(abs(br_diff)>impossible_encoder_diff)
	{
		if (br_prev > encoder_max_count/2) br_diff = (encoder_max_count-br_prev)+br;
		else br_diff = -br_prev-(encoder_max_count-br);
		if(abs(br_diff)>spike_diff) br_diff = 0;
	}
}

void Encoders::calculateWheelDistancesFromEncoders()
{
	if(this->counter_left_prev != this->counter_left)
	{
		fl_dist = (double)fl_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
		bl_dist = (double)bl_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
	}
	else
	{
		fl_dist=0;
		bl_dist=0;
	}
	this->counter_left_prev = this->counter_left;

	if(this->counter_right_prev != this->counter_right)
	{
		fr_dist = (double)fr_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
		br_dist = (double)br_diff/(double)counts_per_revolution*2.0*wheel_radius*3.14159265;
	}
	else
	{
		fr_dist=0;
		br_dist=0;
	}
	this->counter_right_prev = this->counter_right;
}

void Encoders::calculateDeltaDistance4Wheels(int turnFlag, int stopFlag)
{
	short int logical = (1-stopFlag)*(1-turnFlag);
	delta_distance  = (double)logical*1/2*( (fl_dist+bl_dist)/2 + (fr_dist+br_dist)/2 );
}
