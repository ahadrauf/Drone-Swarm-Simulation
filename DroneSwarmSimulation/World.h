/*
Defines the world the drones operate in as a rectangle of dimensions [-XLENGTH/2, XLENGTH/2] x [-YLENGTH/2, YLENGTH/2]
	(ZLENGTH [max altitude] is assumed to be infinite)

TODO: Divide the world into an NxN grid of operating points, each with their own wind speed/heading, altitude, etc.
	Note: It'll be fine if the drones leave the grid, they'll just be operating in a world of zero wind speed, zero ground elevation, etc.
*/

#ifndef WORLD_H
#define WORLD_H
#include <vector>
#include "Drone.h"
using namespace std;

enum HEADING { NORTH, SOUTH, EAST, WEST, NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST };

class World {
public: 
	World(double _xLength, double _yLength, double _zLength, double _windSpeed, HEADING _windHeading, double _T) :
			xLength(_xLength), yLength(_yLength), zLength(_zLength), windSpeed(_windSpeed), windHeading(_windHeading), T(_T), 
			commandsAssigned(true) { }

	World(double _xLength, double _yLength, double _zLength, double _windSpeed, HEADING _windHeading, double _T, Drone* _drones[], int _numDrones) :
			xLength(_xLength), yLength(_yLength), zLength(_zLength), windSpeed(_windSpeed), windHeading(_windHeading), T(_T),
			commandsAssigned(true) {
		for (int i = 0; i < _numDrones; i++) {
			drones.push_back(_drones[i]);
		}
	}

	//Initialization (allows you to add new drones to the world)
	void addDrone(Drone *d) { drones.push_back(d); }

	//Main features
	void update(); //Updates all the drone states
	void sendCommand(Command c); //Queue a command to be implemented during the next update() cycle
	void updateCommand(int id, Command c); //Tells all the drones currently a command with the given id to do the new specified command
	void removeCommand(int id); //Removes all commands with the given id
	void stop(); //Removes all previously executed commands

	vector<Command> getCommands() { return commands; }

	//Helper functions
	Drone* getDrone(int index) { return drones[index]; }
	double getXLength() { return xLength; }
	double getYLength() { return yLength; }
	double getZLength() { return zLength; }
	double getWindSpeed() { return windSpeed; }
	HEADING getWindHeading() { return windHeading; }
	double getT() { return T; }

private:
	double xLength, yLength, zLength; //dimensions in meters
	double windSpeed; //in meters/second
	HEADING windHeading; //i.e. direction wind is going to
	vector<Drone *> drones;
	vector<Command> commands; //stores the commands yet to be implemented
	double T; //time resolution = time/cycle for simulations (in s); also functions as effective drone time delay between actions

	void assignCommands(); //optimally assigns the commands in COMMANDQUEUE to all the drones in DRONES
	bool commandsAssigned; //true if all commands are currently assigned

	void targetOperation(Drone *d, double xPos, double yPos, double zPos); //helper function
	void circleOperation(Drone *d, double xPos, double yPos, double zPos, double turnTime, bool clockwise); //helper function

	//Currently in progress, not implemented
	double effectiveXAccel(double xAccel, double MAX_XY_ACCEL); //actual acceleration the drone would take factoring in wind
	double effectiveYAccel(double yAccel, double MAX_XY_ACCEL); //actual acceleration the drone would take factoring in wind
};

#endif