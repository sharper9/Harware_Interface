#include <robot_control/drive_pivot.h>

void DrivePivot::init()
{
	initHeading_ = robotStatus.heading;
	desiredDeltaHeading_ = params.float1;
    timeoutValue_ = (unsigned int)round((10.0 + fabs(desiredDeltaHeading_)/10.0)*robotStatus.loopRate);
	timeoutCounter_ = 0;
	rSpeedI_ = 0.0;
	inThreshold_ = false;
	thresholdTime_ = 0.0;
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = true;
    yawRatePrev_ = 0.0;
}

int DrivePivot::run()
{
    //int temp;
    ROS_INFO_THROTTLE(1,"drive pivot running");
    //ROS_INFO("======================");
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = true;
    rMax_ = robotStatus.rMax;
	deltaHeading_ = robotStatus.heading - initHeading_;
    ROS_INFO("desiredDeltaHeading = %f", desiredDeltaHeading_);
    ROS_INFO("deltaHeading_ = %f",deltaHeading_);
	rDes_ = kpR_*(desiredDeltaHeading_-deltaHeading_);
	if(rDes_>rMax_) rDes_ = rMax_;
	else if(rDes_<(-rMax_)) rDes_ = -rMax_;
    if(std::isnan(robotStatus.yawRate)) {ROS_ERROR("yaw rate is nan"); robotStatus.yawRate = yawRatePrev_;}
    else yawRatePrev_ = robotStatus.yawRate;
    ROS_INFO("yawRate = %f",robotStatus.yawRate);
	errorR_ = rDes_ - robotStatus.yawRate;
    //ROS_INFO("errorR_ = %f",errorR_);
	rSpeedP_ = kROutput_*rDes_;
    //ROS_INFO("rSpeedP_ = %f",rSpeedP_);
    rSpeedI_ += kiR_*errorR_;
    //ROS_INFO("rSpeedI_ before coerc = %f",rSpeedI_);
    if(rSpeedI_>rSpeedIMax_) rSpeedI_ = rSpeedIMax_;
    else if(rSpeedI_<-rSpeedIMax_) rSpeedI_ = -rSpeedIMax_;
    //ROS_INFO("rSpeedI_ before coerc = %f",rSpeedI_);
    rSpeedT_ = round(rSpeedP_ + rSpeedI_);
    //ROS_INFO("rSpeedT_ before coerc = %f",rSpeedT_);
	if(rSpeedT_>rSpeedMax_) rSpeedT_ = rSpeedMax_;
	else if(rSpeedT_<(-rSpeedMax_)) rSpeedT_ = -rSpeedMax_;
    //ROS_INFO("rSpeedT after coerc = %f",rSpeedT_);
    //ROS_INFO("rDes: %f",rDes_);
    //ROS_INFO("deltaHeading = %f", deltaHeading_);
    //ROS_INFO("desiredDeltaHeading_-deltaHeading_: %f", desiredDeltaHeading_-deltaHeading_);
	leftSpeed_ = rSpeedT_;
	rightSpeed_ = -rSpeedT_;
    //ROS_INFO("leftSpeed_ = %f",leftSpeed_);
    //ROS_INFO("rightSpeed_ = %f",rightSpeed_);
	timeoutCounter_++;
	if(fabs(desiredDeltaHeading_-deltaHeading_)<=deltaHeadingThreshold_ && inThreshold_==false) {thresholdInitTime_ = ros::Time::now().toSec(); inThreshold_ = true;}
	if(fabs(desiredDeltaHeading_-deltaHeading_)<=deltaHeadingThreshold_) thresholdTime_ = ros::Time::now().toSec() - thresholdInitTime_;
	else {thresholdTime_ = 0.0; inThreshold_ = false;}
	if(thresholdTime_ >= thresholdMinTime_ || timeoutCounter_ >= timeoutValue_)
	{
		robotOutputs.flMotorSpeed = 0;
		robotOutputs.blMotorSpeed = 0;
		robotOutputs.frMotorSpeed = 0;
		robotOutputs.brMotorSpeed = 0;
        ROS_INFO("end pivot");
		taskEnded_ = 1;
	}
	else
	{
		robotOutputs.flMotorSpeed = (int)round(leftSpeed_);
		robotOutputs.blMotorSpeed = (int)round(leftSpeed_);
		robotOutputs.frMotorSpeed = (int)round(rightSpeed_);
		robotOutputs.brMotorSpeed = (int)round(rightSpeed_);
		taskEnded_ = 0;
	}
    //ROS_INFO("task ended = %i",taskEnded_);
    //std::printf("\n");
	return taskEnded_;
}
