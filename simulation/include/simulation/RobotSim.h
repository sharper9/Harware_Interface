#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H
#include <math.h>
#include <stdint.h>
#include <ros/ros.h>

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

#define SCOOP_RAISED -1000
#define SCOOP_LOWERED 0
#define ARM_RAISED 1000
#define ARM_LOWERED -900
#define ARM_DUMP 0
#define BUCKET_RAISED 1000
#define BUCKET_LOWERED -1000

class RobotSim
{
public:
	// Members
	// Drive
	double xPos; // m
	double yPos; // m
	double heading; // deg
    bool leftBumper;
    bool rightBumper;
    // Linear Actuators
    int scoopPos;
    int scoopPosCmdPrev;
    int armPos;
    int armPosCmdPrev;
    int bucketPos;
    int bucketPosCmdPrev;
    int scoopStop;
    int armStop;
    int bucketStop;
	// Sim
	double normalSpeedDT = 0.05;  // Default of 20 Hz, resulting in 1x speed
	double dt = normalSpeedDT;
	// Methods
	RobotSim(double initX, double initY, double initHeading, double simRate); // Constructor
	void drive(double linV, double angV); // Drive robot using linear and angular velocities as input. Arg units: m/s, deg/s
	void teleport(double teleX, double teleY, double teleHeading); // Teleport robot to location. Arg units: m, m, deg
    void runLinearActuators(int scoopPosCmd, int armPosCmd, int bucketPosCmd, int scoopStopCmd, int armStopCmd, int bucketStopCmd); // Manage linear actuator operations
private:
    const double scoopSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
    const double armSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
    const double bucketSpeed_ = 2000.0/5.0; // full range of -1000 to 1000 in 5 seconds
};

#endif // ROBOT_SIM_H
