#include <robot_control/action.h>

void Action::pushTask(TASK_TYPE_T taskType)
{
    if(taskType==_driveHalt_ || taskType==_driveStraight_ || taskType==_pivot_ || taskType==_driveUntilLimit_)
		driveDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
    else if(taskType==_scoopHalt_ || taskType==_scoopSetPos_)
        scoopDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
    else if(taskType==_armHalt_ || taskType==_armSetPos_ || taskType==_armShake_)
        armDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
    else if(taskType==_bucketHalt_ || taskType==_bucketSetPos_ || taskType==_bucketShake_)
        bucketDeque.push_back(taskPool[taskType][taskPoolIndex[taskType]]);
	else ROS_ERROR("attempted to push back invalid TASK type");
	taskPoolIndex[taskType]++;
    if(taskPoolIndex[taskType]>=TASK_POOL_SIZE) taskPoolIndex[taskType] = 0;
}

int Action::runDeques()
{
	driveDequeEmpty = driveDeque.empty();
	if(driveDequeEmptyPrev && !driveDequeEmpty) driveDeque.front()->init();
    else if(!driveDequeEmptyPrev && driveDequeEmpty) driveHalt.init();
    if(driveDequeEmpty) {driveHalt.run(); driveDequeEmpty = 1; driveDequeEnded = 0;}
	else driveDequeEnded = driveDeque.front()->run();
	if(driveDequeEnded!=0)
	{
		driveDeque.pop_front();
		driveDequeEmpty = driveDeque.empty();
		if(driveDequeEmpty==0) driveDeque.front()->init();
	}

    scoopDequeEmpty = scoopDeque.empty();
    if(scoopDequeEmptyPrev && !scoopDequeEmpty) scoopDeque.front()->init();
    if(scoopDequeEmpty) {scoopHalt.run(); scoopDequeEmpty = 1; scoopDequeEnded = 0;}
    else scoopDequeEnded = scoopDeque.front()->run();
    if(scoopDequeEnded!=0)
	{
        if(scoopFailed) scoopDeque.clear();
        else scoopDeque.pop_front();
        scoopDequeEmpty = scoopDeque.empty();
        if(scoopDequeEmpty==0) scoopDeque.front()->init();
	}

    armDequeEmpty = armDeque.empty();
    if(armDequeEmptyPrev && !armDequeEmpty) armDeque.front()->init();
    if(armDequeEmpty) {armHalt.run(); armDequeEmpty = 1; armDequeEnded = 0;}
    else armDequeEnded = armDeque.front()->run();
    if(armDequeEnded!=0)
    {
        if(armFailed) armDeque.clear();
        else armDeque.pop_front();
        armDequeEmpty = armDeque.empty();
        if(armDequeEmpty==0) armDeque.front()->init();
    }

    bucketDequeEmpty = bucketDeque.empty();
    if(bucketDequeEmptyPrev && !bucketDequeEmpty) bucketDeque.front()->init();
    if(bucketDequeEmpty) {bucketHalt.run(); bucketDequeEmpty = 1; bucketDequeEnded = 0;}
    else bucketDequeEnded = bucketDeque.front()->run();
    if(bucketDequeEnded!=0)
    {
        if(bucketFailed) bucketDeque.clear();
        else bucketDeque.pop_front();
        bucketDequeEmpty = bucketDeque.empty();
        if(bucketDequeEmpty==0) bucketDeque.front()->init();
    }

	driveDequeEmptyPrev = driveDequeEmpty;
    scoopDequeEmptyPrev = scoopDequeEmpty;
    armDequeEmptyPrev = armDequeEmpty;
    bucketDequeEmptyPrev = bucketDequeEmpty;
    if(driveDequeEmpty && scoopDequeEmpty && armDequeEmpty && bucketDequeEmpty) return 1;
	else return 0;
}

void Action::clearDeques()
{
    driveDeque.clear();
    scoopDeque.clear();
    armDeque.clear();
    bucketDeque.clear();
    //driveDequeEmptyPrev = 1;
    //scoopDequeEmptyPrev = 1;
    //armDequeEmptyPrev = 1;
}

void Action::initDequesFront()
{
    if(!driveDeque.empty()) driveDeque.front()->init();
    if(!scoopDeque.empty()) scoopDeque.front()->init();
    if(!armDeque.empty()) armDeque.front()->init();
    if(!bucketDeque.empty()) bucketDeque.front()->init();
}
