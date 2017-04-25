#include <robot_control/wait.h>

Wait::Wait()
{
	waitTimer = new CataglyphisTimer<Wait>(&Wait::waitTimeCallback_, this);
	waitTimerActive = false;
}

void Wait::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
	waitTime_ = params.float1;
	timeExpired_ = false;
	waitTimer->stop();
	waitTimer->setPeriod(waitTime_);
	clearDeques();
	driveHalt.init();
    scoopHalt.init();
    armHalt.init();
    bucketHalt.init();
	waitTimer->start();
	waitTimerActive = true;
}

int Wait::run()
{
	driveHalt.run();
    scoopHalt.run();
    armHalt.run();
    bucketHalt.run();
	if(driveDeque.empty()) driveDequeEmptyPrev = 1;
    if(scoopDeque.empty()) scoopDequeEmptyPrev = 1;
    if(armDeque.empty()) armDequeEmptyPrev = 1;
    if(bucketDeque.empty()) bucketDequeEmptyPrev = 1;
	return timeExpired_;
}

void Wait::waitTimeCallback_(const ros::TimerEvent &event)
{
	timeExpired_ = true;
	waitTimerActive = false;
}
