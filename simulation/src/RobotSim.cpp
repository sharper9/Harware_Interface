#include <simulation/RobotSim.h>

RobotSim::RobotSim(double initX, double initY, double initHeading, double simRate)
{
	teleport(initX, initY, initHeading);
    normalSpeedDT = 1.0/simRate;
    dt = normalSpeedDT;
    scoopPos = SCOOP_RAISED;
    armPos = ARM_LOWERED;
    bucketPos = BUCKET_LOWERED;
    scoopPosCmdPrev = SCOOP_RAISED;
    armPosCmdPrev = ARM_LOWERED;
    scoopStop = 1;
    armStop = 1;
    bucketStop = 1;
    leftBumper = false;
    rightBumper = false;
}

void RobotSim::drive(double linV, double angV)
{
    heading = heading + angV*dt;
    xPos = xPos + linV*cos(heading*DEG2RAD)*dt;
    yPos = yPos + linV*sin(heading*DEG2RAD)*dt;
    if(xPos<0.2)
    {
        leftBumper = true;
        rightBumper = true;
    }
    else
    {
        leftBumper = false;
        rightBumper = false;
    }
}

void RobotSim::teleport(double teleX, double teleY, double teleHeading)
{
    xPos = teleX;
    yPos = teleY;
    heading = teleHeading;
}

void RobotSim::runLinearActuators(int scoopPosCmd, int armPosCmd, int bucketPosCmd, int scoopStopCmd, int armStopCmd, int bucketStopCmd)
{
    //ROS_INFO("scoopCmd = %i, armCmd = %i, bucketCmd = %i",scoopPosCmd, armPosCmd, bucketPosCmd);
    //ROS_INFO("scoopPos = %i, armPos = %i, bucketPos = %i",scoopPos, armPos, bucketPos);
    //ROS_INFO("scoopStop = %i, armStop = %i, bucketStop = %i",scoopStop, armStop, bucketStop);
    if(scoopPosCmd!=scoopPosCmdPrev && scoopStopCmd==0) scoopStop = 0;
    if(armPosCmd!=armPosCmdPrev && armStopCmd==0) armStop = 0;
    if(bucketPosCmd!=bucketPosCmdPrev && bucketStopCmd==0) bucketStop = 0;
    if(scoopStop==0)
    {
        if(fabs(scoopPos-scoopPosCmd)<=(int)round(scoopSpeed_*dt)) scoopPos = scoopPosCmd;
        if(scoopPosCmd > scoopPos) scoopPos += (int)round(scoopSpeed_*dt);
        else if(scoopPosCmd < scoopPos) scoopPos -= (int)round(scoopSpeed_*dt);
        if(scoopPos==scoopPosCmd || scoopStopCmd) scoopStop = 1;
    }
    if(armStop==0)
    {
        if(fabs(armPos-armPosCmd)<=(int)round(armSpeed_*dt)) armPos = armPosCmd;
        if(armPosCmd > armPos) armPos += (int)round(armSpeed_*dt);
        else if(armPosCmd < armPos) armPos -= (int)round(armSpeed_*dt);
        if(armPos==armPosCmd || armStopCmd) armStop = 1;
    }
    if(bucketStop==0)
    {
        if(fabs(bucketPos-bucketPosCmd)<=(int)round(bucketSpeed_*dt)) bucketPos = bucketPosCmd;
        if(bucketPosCmd > bucketPos) bucketPos += (int)round(bucketSpeed_*dt);
        else if(bucketPosCmd < bucketPos) bucketPos -= (int)round(bucketSpeed_*dt);
        if(bucketPos==bucketPosCmd || bucketStopCmd) bucketStop = 1;
    }
    if(scoopStopCmd) scoopPosCmdPrev = scoopPos;
    else scoopPosCmdPrev = scoopPosCmd;
    if(armStopCmd) armPosCmdPrev = armPos;
    else armPosCmdPrev = armPosCmd;
    if(bucketStopCmd) bucketPosCmdPrev = bucketPos;
    else bucketPosCmdPrev = bucketPosCmd;
}
