#include "Drone.h"
#include <iostream>
using namespace std;


//Constructor
//Calculated for the AR Drone 2.0 using one battery
//https://en.wikipedia.org/wiki/Parrot_AR.Drone
//Efficiency value calculated so a constant 5 m/s^2 acceleration drains the battery in about 18-20 minutes
Drone::Drone() : MAX_XY_VEL(AR_DRONE_2_MAX_XY_VEL), MAX_Z_VEL(AR_DRONE_2_MAX_Z_VEL), MAX_Z_POS(AR_DRONE_2_MAX_Z_POS), 
		MAX_XY_ACCEL(AR_DRONE_2_MAX_XY_ACCEL), MAX_Z_ACCEL(AR_DRONE_2_MAX_Z_ACCEL), MASS(AR_DRONE_2_MASS),
		PROPELLER_RADIUS(AR_DRONE_2_PROPELLER_RADIUS), MAX_BATTERY_LIFE(AR_DRONE_2_MAX_BATTERY_LIFE),  EFFICIENCY(AR_DRONE_2_EFFICIENCY) {
	xPos = yPos = zPos = 0;
	xVel = yVel = zVel = 0;
	xAccel = yAccel = zAccel = 0;
	batteryLife = MAX_BATTERY_LIFE;
	alive = 1;
	command.option = Command::NONE;
	locked = false;
}

Drone::Drone(double _xPos, double _yPos) : Drone() {
	xPos = _xPos;
	yPos = _yPos;
}

/*
Drone::Drone(const Drone &d) : MAX_XY_VEL(AR_DRONE_2_MAX_XY_VEL), MAX_Z_VEL(AR_DRONE_2_MAX_Z_VEL), MAX_Z_POS(AR_DRONE_2_MAX_Z_POS),
		MAX_XY_ACCEL(AR_DRONE_2_MAX_XY_ACCEL), MAX_Z_ACCEL(AR_DRONE_2_MAX_Z_ACCEL), MASS(AR_DRONE_2_MASS),
		PROPELLER_RADIUS(AR_DRONE_2_PROPELLER_RADIUS), MAX_BATTERY_LIFE(AR_DRONE_2_MAX_BATTERY_LIFE), EFFICIENCY(AR_DRONE_2_EFFICIENCY) {
	xPos = d.xPos;
	yPos = d.yPos;
	zPos = d.zPos;
	xVel = d.xVel;
	yVel = d.yVel;
	zVel = d.zVel;
	xAccel = d.xAccel;
	yAccel = d.yAccel;
	zAccel = d.zAccel;
	batteryLife = d.batteryLife;
	alive = d.alive;
	command.option = Command::NONE;
}
*/

/**
Computes the power consumption (in Watts) of the drone at the current time
Computed according to the formula P = K*(m^(3/2)) / r, where
K = 22.35 (assuming 1atm and 20 degrees Celcius)
m = thrust required in kg = m + F/g = (DRONE_MASS*ACCELERATION) / g
r = propeller radius in meters
Equation gotten from: https://justdrones.com.au/how-much-power-is-needed-to-hover/
**/
double Drone::power() {
	if (zPos == 0) { //if the drone is on the ground
		return 0;
	}
	double accel = sqrt(xAccel*xAccel + yAccel*yAccel + (zAccel + 9.8) * (zAccel + 9.8));
	double thrust = MASS * accel / 9.8;
	return 22.35 * pow(thrust, 1.5) / PROPELLER_RADIUS / EFFICIENCY;
}

void Drone::setCommand(struct Command c) {
	command = c;
}

void Drone::reset() {
	xPos = yPos = zPos = 0;
	xVel = yVel = zVel = 0;
	xAccel = yAccel = zAccel = 0;
	batteryLife = MAX_BATTERY_LIFE;
	alive = 1;
	command.option = Command::NONE;
	locked = false;
}

void Drone::setXVel(double _xVel) { 
	xVel = _xVel < MAX_XY_VEL ? _xVel : MAX_XY_VEL;
}
void Drone::setYVel(double _yVel) { 
	yVel = _yVel < MAX_XY_VEL ? _yVel : MAX_XY_VEL;
	//cout << yVel << endl;
}
void Drone::setZVel(double _zVel) { 
	zVel = _zVel < MAX_Z_VEL ? _zVel : MAX_Z_VEL;
}
void Drone::setXAccel(double _xAccel) { 
	xAccel = _xAccel < MAX_XY_ACCEL ? _xAccel : MAX_XY_ACCEL;
}
void Drone::setYAccel(double _yAccel) { 
	yAccel = _yAccel < MAX_XY_ACCEL ? _yAccel : MAX_XY_ACCEL;
	//cout << yAccel << endl;
}
void Drone::setZAccel(double _zAccel) { 
	zAccel = _zAccel < MAX_Z_ACCEL ? _zAccel : MAX_Z_ACCEL;
}