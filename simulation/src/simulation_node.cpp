#include <ros/ros.h>
#include <simulation/RobotSim.h>
#include <messages/ActuatorOut.h>
#include <messages/NavFilterOut.h>
#include <messages/SimControl.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/NavFilterControl.h>
#include <hw_interface_plugin_roboteq/Roboteq_Data.h>
#include <hw_interface_plugin_agent/pause.h>
#include <chrono>
#include <random>

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg);
void simControlCallback(const messages::SimControl::ConstPtr& msg);
void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg);

const double simRate = 50.0; // Hz
messages::ActuatorOut actuatorCmd;
RobotSim robotSim(1.0, 0.0, 0.0,simRate);
hw_interface_plugin_agent::pause pauseMsg;

int main(int argc, char** argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::Subscriber actuatorSub = nh.subscribe<messages::ActuatorOut>("control/actuatorout/all",1,actuatorCallback);
    ros::Subscriber simConSub = nh.subscribe<messages::SimControl>("simulation/simcontrol/simcontrol",1,simControlCallback);
    ros::Publisher navPub = nh.advertise<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
    ros::Publisher scoopPub = nh.advertise<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/brushed/wrist",1);
    ros::Publisher armPub = nh.advertise<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/brushed/arm",1);
    ros::Publisher bucketPub = nh.advertise<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/brushed/bucket",1);
    ros::Publisher pausePub = nh.advertise<hw_interface_plugin_agent::pause>("/agent/pause",1);
    ros::Publisher leftDrivePub = nh.advertise<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/drivemotorin/left",1);
    ros::Publisher rightDrivePub = nh.advertise<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/drivemotorin/right",1);
    messages::NavFilterOut navMsgOut;
    hw_interface_plugin_roboteq::Roboteq_Data scoopFeedbackMsg;
    hw_interface_plugin_roboteq::Roboteq_Data armFeedbackMsg;
    hw_interface_plugin_roboteq::Roboteq_Data bucketFeedbackMsg;
    messages::nb1_to_i7_msg nb1MsgOut;
    hw_interface_plugin_roboteq::Roboteq_Data leftDriveFeedbackMsg;
    hw_interface_plugin_roboteq::Roboteq_Data rightDriveFeedbackMsg;

    double linV; // m/s
    double angV; // deg/s
    const double linVGain = 1.2/900.0/4.0; // m/s per speed cmd
    const double angVGain = 45.0/2650.0; // deg/s per speed cmd
    double prevPubTime = 0.0;
    double prevLoopTime;
    double currentLoopTime;
    double deltaLoopTime;
    actuatorCmd.wrist_pos_cmd = SCOOP_RAISED;
    actuatorCmd.arm_pos_cmd = ARM_LOWERED;
    actuatorCmd.bucket_pos_cmd = BUCKET_LOWERED;
    actuatorCmd.wrist_stop_cmd = 0;
    actuatorCmd.arm_stop_cmd = 0;
    actuatorCmd.bucket_stop_cmd = 0;
    pauseMsg.pause = true; // Initialize pause to true
    scoopFeedbackMsg.feedback.resize(2);
    armFeedbackMsg.feedback.resize(2);
    bucketFeedbackMsg.feedback.resize(2);
    leftDriveFeedbackMsg.individual_digital_inputs.resize(6);
    rightDriveFeedbackMsg.individual_digital_inputs.resize(6);

    ros::Rate loopRate(simRate);
    prevLoopTime = ros::Time::now().toSec();

    while(ros::ok())
    {
        currentLoopTime = ros::Time::now().toSec();
        deltaLoopTime = currentLoopTime - prevLoopTime;
        prevLoopTime = currentLoopTime;
        //ROS_INFO_THROTTLE(1.0,"deltaLoopTime = %f",deltaLoopTime);
        linV = linVGain*(actuatorCmd.fl_speed_cmd + actuatorCmd.fr_speed_cmd + actuatorCmd.bl_speed_cmd + actuatorCmd.br_speed_cmd);
        angV = angVGain*(actuatorCmd.fl_speed_cmd - actuatorCmd.fr_speed_cmd + actuatorCmd.bl_speed_cmd - actuatorCmd.br_speed_cmd);
        //ROS_INFO("linV: %f",linV);
        //ROS_INFO("angV: %f",angV);
        robotSim.drive(linV, angV);
        robotSim.runLinearActuators(actuatorCmd.wrist_pos_cmd, actuatorCmd.arm_pos_cmd, actuatorCmd.bucket_pos_cmd, actuatorCmd.wrist_stop_cmd, actuatorCmd.arm_stop_cmd, actuatorCmd.bucket_stop_cmd);
        scoopFeedbackMsg.feedback.at(0) = robotSim.scoopPos;
        scoopFeedbackMsg.feedback.at(1) = robotSim.scoopPos;
        armFeedbackMsg.feedback.at(0) = robotSim.armPos;
        armFeedbackMsg.feedback.at(1) = robotSim.armPos;
        bucketFeedbackMsg.feedback.at(0) = robotSim.bucketPos;
        bucketFeedbackMsg.feedback.at(1) = robotSim.bucketPos;
        leftDriveFeedbackMsg.individual_digital_inputs.at(5) = robotSim.leftBumper;
        rightDriveFeedbackMsg.individual_digital_inputs.at(5) = robotSim.rightBumper;
        navMsgOut.x_position = robotSim.xPos;
        navMsgOut.y_position = robotSim.yPos;
        navMsgOut.velocity = linV;
        navMsgOut.yaw_rate = angV;
        navMsgOut.heading = robotSim.heading;
        navMsgOut.human_heading = fmod(robotSim.heading, 360.0);
        navPub.publish(navMsgOut);
        scoopPub.publish(scoopFeedbackMsg);
        armPub.publish(armFeedbackMsg);
        bucketPub.publish(bucketFeedbackMsg);
        pausePub.publish(pauseMsg);
        leftDrivePub.publish(leftDriveFeedbackMsg);
        rightDrivePub.publish(rightDriveFeedbackMsg);
        loopRate.sleep();
        ros::spinOnce();
    }
    return 0;
}

void actuatorCallback(const messages::ActuatorOut::ConstPtr& msg)
{
    actuatorCmd = *msg;
}

void simControlCallback(const messages::SimControl::ConstPtr& msg)
{
    if(msg->teleport)
    {
        robotSim.teleport(msg->teleX, msg->teleY, msg->teleHeading);
    }
    if(msg->setSimSpeed)
    {
        if(msg->simSpeed>0.0) robotSim.dt = robotSim.normalSpeedDT*msg->simSpeed;
    }
    pauseMsg.pause = msg->pauseSwitch;
}

void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg)
{
    newX = origX*cos(DEG2RAD*angleDeg)+origY*sin(DEG2RAD*angleDeg);
    newY = -origX*sin(DEG2RAD*angleDeg)+origY*cos(DEG2RAD*angleDeg);
}
