#ifndef DRONE_H
#define DRONE_H
#include <math.h>
#include <iostream>
using namespace std;

/**
Defines a DRONE object, the COMMAND struct, and AR_DRONE_2 parameters
XY plane = plane of horizontal motion
Z axis = vertical motion
**/



#ifndef AR_DRONE_2
#define AR_DRONE_2
#define AR_DRONE_2_MAX_XY_VEL 25*30
#define AR_DRONE_2_MAX_Z_VEL 25
#define AR_DRONE_2_MAX_Z_POS 1000
#define AR_DRONE_2_MAX_XY_ACCEL 5*10
#define AR_DRONE_2_MAX_Z_ACCEL 5
#define AR_DRONE_2_MASS 0.320 //in kg
#define AR_DRONE_2_PROPELLER_RADIUS 0.2 //in m
#define AR_DRONE_2_MAX_BATTERY_LIFE 59760 //in J
#define AR_DRONE_2_EFFICIENCY 0.4
#endif

/*
Defines the structure of a command sent to the drone
Ex. If option == X_ACCEL and value = 5, the drone should try to change it's x-acceleration by 5

TARGET option is a custom setting for modeling drone formations; it tells the drone to focus on approaching a target
*/
struct Position {
	double xPos, yPos, zPos;
};

struct Command {
	int id;
	enum OPTIONS { NONE, X_ACCEL, Y_ACCEL, Z_ACCEL, TARGET, RETURN_TO_BASE, CIRCLE, SURROUND_AND_CIRCLE, SURROUND_AND_HOVER, SURROUND_AND_HOVER_AVOID_ANGLE } option;
	unsigned int priority; //higher priority commands have larger values
	bool activeCommand; //= false only after the command gets removed, generally don't play around with this setting

	union {
		double value; //for X/Y/Z-ACCEL command
		struct Position position;
		struct Turn { //assumes you're at the desired radius from the circle
			struct Position position;
			double radius; //in m
			double turnTime; //in s
			bool counterclockwise; //looking down from the top
		} turn;
		struct SurroundAndCircle {
			struct Position position;
			double innerRadius; //in m, the drone crashes if it goes within this radius
			double outerRadius; //target distance from the object
			double turnTime; //in s
			bool counterclockwise; //looking down from the top
		} surroundAndCircle;
		struct SurroundAndHover {
			struct Position position;
			double innerRadius; //in m, the drone crashes if it goes within this radius
			double outerRadius; //target distance from the object
			double turnTime; //in s
			bool counterclockwise; //looking down from the top
		} surroundAndHover;
		struct SurroundAndHoverAvoidAngle {
			struct Position position;
			double innerRadius; //in m, the drone crashes if it goes within this radius
			double outerRadius; //target distance from the object
			double turnTime; //in s
			bool counterclockwise; //looking down from the top
			double angleToAvoid; //in degrees, oriented like a Cartesian plane (0 = to the right)
			double angleTolerance; //in degrees, drones shouldn't be within [angleToAvoid +/- angleTolerance] region
		} surroundAndHoverAvoidAngle;
	} info;
};

class Drone {
public:
	Drone();
	Drone(double _xPos, double _yPos);
	//Drone(const Drone &d);

	/**
	Computes the power consumption of the drone at the current time
	**/
	double power();
	void reset(); //reset the state back to initial conditions (just like calling the constructor again)

	void setXPos(double _xPos) { xPos = _xPos; }
	void setYPos(double _yPos) { yPos = _yPos; }
	void setZPos(double _zPos) { zPos = _zPos > MAX_Z_POS ? MAX_Z_POS : (_zPos < 0) ? 0 : _zPos; }
	void setXVel(double _xVel);
	void setYVel(double _yVel);
	void setZVel(double _zVel);
	void setXAccel(double _xAccel);
	void setYAccel(double _yAccel);
	void setZAccel(double _zAccel);
	void setCommand(struct Command c);
	void setBatteryLife(double _batteryLife) { batteryLife = _batteryLife; }
	void setAlive(int _alive) { alive = _alive; }
	void setLocked(bool _locked) { locked = _locked; }

	double getXPos() { return xPos; }
	double getYPos() { return yPos; }
	double getZPos() { return zPos; }
	double getXVel() { return xVel; }
	double getYVel() { return yVel; }
	double getZVel() { return zVel; }
	double getXAccel() { return xAccel; }
	double getYAccel() { return yAccel; }
	double getZAccel() { return zAccel; }
	double getBatteryLife() { return batteryLife; }
	int getAlive() { return alive; }
	struct Command *getCommand() { return &command; }
	double getMaxXYVel() { return MAX_XY_VEL; }
	double getMaxZVel() { return MAX_Z_VEL; }
	double getMaxXYAccel() { return MAX_XY_ACCEL; }
	double getMaxZAccel() { return MAX_Z_ACCEL; }
	double getMaxZPos() { return MAX_Z_POS; }
	double getMaxBatteryLife() { return MAX_BATTERY_LIFE; }
	bool getLocked() { return locked; }
private:
	double xPos, yPos, zPos;
	double xVel, yVel, zVel;
	double xAccel, yAccel, zAccel;
	double batteryLife; //in J
	int alive; //default = 1, turns to 0 if the drone crashes with another drone
	struct Command command;
	bool locked; //= false by default, = true when you want to lock the drone into position (ex. for surroundAndHover)

	const double MAX_XY_VEL, MAX_Z_VEL; //in m/s
	const double MAX_XY_ACCEL, MAX_Z_ACCEL; //in m/s^2
	const double MAX_Z_POS; //aka max altitude, in m
	const double MASS; //in kg
	const double PROPELLER_RADIUS; //in m
	const double MAX_BATTERY_LIFE; //in J
	const double EFFICIENCY; //for power consumption
};

#endif