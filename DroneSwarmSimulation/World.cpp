#include "World.h"
#include <limits>
#include <omp.h>

//Queue a command to be implemented during the next update() cycle
void World::sendCommand(Command c) {
	c.activeCommand = true;
	commands.push_back(c); 
	commandsAssigned = false;
}

//Tells all the drones currently doing commands[index] to do the new specified command
void World::updateCommand(int id, Command c) {
	bool addedCommand = false;
	for (unsigned int i = 0; i < drones.size(); i++) {
		if (drones[i]->getCommand()->id == id) {
			c.activeCommand = true;
			drones[i]->setCommand(c);
			drones[i]->setLocked(false);
		}
	}
	for (unsigned int i = 0; i < commands.size(); i++) {
		if (commands[i].id == id) {
			commands[i] = c;
		}
	}
}

//Removes the command with the given index
void World::removeCommand(int id) { 
	//commands.erase(commands.begin() + index);
	for (unsigned int i = 0; i < commands.size(); i++) {
		if (commands[i].id == id) {
			commands[i].activeCommand = false;
			commandsAssigned = false;
		}
	}
}

void World::stop() {
	for (unsigned int i = 0; i < commands.size(); i++) {
		commands[i].activeCommand = false;
		commandsAssigned = false;
	}
}

void World::update() {
	assignCommands(); //Update command list

	const int NUM_DRONES = drones.size();
	#pragma omp parallel for
	for (int i = 0; i < NUM_DRONES; i++) {
		//if (i == 50) cout << i << endl;

		if (drones[i]->getAlive() == 0) { //don't update dead drones + have them crash
			drones[i]->setZPos(0);
			continue;
		}
		
		double xAccel = drones[i]->getXAccel(); //effectiveXAccel(drones[i]->getXAccel(), drones[i]->getMaxXYAccel());
		double yAccel = drones[i]->getYAccel(); //effectiveYAccel(drones[i]->getYAccel(), drones[i]->getMaxXYAccel());

		//Reassign drone positions
		//x' = x + vt + 0.5at^2
		drones[i]->setXPos(drones[i]->getXPos() + drones[i]->getXVel() * T + 0.5 * xAccel * T * T);
		drones[i]->setYPos(drones[i]->getYPos() + drones[i]->getYVel() * T + 0.5 * yAccel * T * T);
		drones[i]->setZPos(drones[i]->getZPos() + drones[i]->getZVel() * T + 0.5 * drones[i]->getZAccel() * T * T);
		if (drones[i]->getZPos() > drones[i]->getMaxZPos()) {
			drones[i]->setZPos(drones[i]->getMaxZPos());
		} else if (drones[i]->getZPos() < 0) {
			drones[i]->setZPos(0);
		}

		//Reassign drone velocities
		//v' = v + at
		drones[i]->setXVel(drones[i]->getXVel() + xAccel * T);
		drones[i]->setYVel(drones[i]->getYVel() + yAccel * T);
		drones[i]->setZVel(drones[i]->getZVel() + drones[i]->getZAccel() * T);

		/*
		//Reduce battery life as necessary
		drones[i]->setBatteryLife(drones[i]->getBatteryLife() - drones[i]->power() * T);
		if (drones[i]->getBatteryLife() < 0) {
			drones[i]->setAlive(0);
		}

		//If drone is low on battery, change active command to RETURN_TO_BASE so it returns to base at (0, 0, 0)
		if (drones[i]->getBatteryLife() < drones[i]->getMaxBatteryLife() * 0.2) {
			drones[i]->getCommand()->option = Command::RETURN_TO_BASE;
			drones[i]->getCommand()->info.position = { 0, 0, 0 };
		}
		*/

		//Reassign drone acceleration from commands
		if (drones[i]->getCommand()->option == Command::X_ACCEL) {
			drones[i]->setXAccel(drones[i]->getCommand()->info.value);
			drones[i]->setYAccel(0);
			drones[i]->setZAccel(0);
		} 
		else if (drones[i]->getCommand()->option == Command::Y_ACCEL) {
			drones[i]->setXAccel(0);
			drones[i]->setYAccel(drones[i]->getCommand()->info.value);
			drones[i]->setZAccel(0);
		} 
		else if (drones[i]->getCommand()->option == Command::Z_ACCEL) {
			//cout << drones[i]->getXVel() << " " << drones[i]->getYVel() << endl;
			drones[i]->setXAccel(0);
			drones[i]->setYAccel(0);
			drones[i]->setZAccel(drones[i]->getCommand()->info.value);
		} 
		else if (drones[i]->getCommand()->option == Command::TARGET || drones[i]->getCommand()->option == Command::RETURN_TO_BASE) {
			targetOperation(drones[i], drones[i]->getCommand()->info.position.xPos, drones[i]->getCommand()->info.position.yPos, 
				drones[i]->getCommand()->info.position.zPos);
		}
		else if (drones[i]->getCommand()->option == Command::CIRCLE) {
			circleOperation(drones[i], drones[i]->getCommand()->info.turn.position.xPos, drones[i]->getCommand()->info.turn.position.yPos,
				drones[i]->getCommand()->info.turn.position.zPos, drones[i]->getCommand()->info.turn.turnTime,
				drones[i]->getCommand()->info.turn.counterclockwise);
		}
		else if (drones[i]->getCommand()->option == Command::SURROUND_AND_CIRCLE) {
			double xPos = drones[i]->getCommand()->info.surroundAndCircle.position.xPos;
			double yPos = drones[i]->getCommand()->info.surroundAndCircle.position.yPos;
			double zPos = drones[i]->getCommand()->info.surroundAndCircle.position.zPos;
			double xDist = drones[i]->getXPos() - xPos;
			double yDist = drones[i]->getYPos() - yPos;
			double zDist = drones[i]->getZPos() - zPos;
			double distance = sqrt(xDist*xDist + yDist*yDist + zDist*zDist);

			if (distance < drones[i]->getCommand()->info.surroundAndCircle.innerRadius) { //drone crashed into target
				drones[i]->setAlive(0);
				continue;
			}
			else if (distance < drones[i]->getCommand()->info.surroundAndCircle.outerRadius) {
				circleOperation(drones[i], drones[i]->getCommand()->info.surroundAndCircle.position.xPos, 
					drones[i]->getCommand()->info.surroundAndCircle.position.yPos,
					drones[i]->getCommand()->info.surroundAndCircle.position.zPos, drones[i]->getCommand()->info.surroundAndCircle.turnTime,
					drones[i]->getCommand()->info.surroundAndCircle.counterclockwise);
			}
			else {
				targetOperation(drones[i], drones[i]->getCommand()->info.surroundAndCircle.position.xPos,
					drones[i]->getCommand()->info.surroundAndCircle.position.yPos,
					drones[i]->getCommand()->info.surroundAndCircle.position.zPos);
			}
		} 
		else if (drones[i]->getCommand()->option == Command::SURROUND_AND_HOVER) {
			double xPos = drones[i]->getCommand()->info.surroundAndHover.position.xPos;
			double yPos = drones[i]->getCommand()->info.surroundAndHover.position.yPos;
			double zPos = drones[i]->getCommand()->info.surroundAndHover.position.zPos;
			double xDist = drones[i]->getXPos() - xPos;
			double yDist = drones[i]->getYPos() - yPos;
			double zDist = drones[i]->getZPos() - zPos;
			double distance = sqrt(xDist*xDist + yDist*yDist + zDist*zDist);
			double angle = atan2(yDist, xDist) * 180 / 3.14;

			/*
			if (distance < drones[i]->getCommand()->info.surroundAndHover.innerRadius) { //drone crashed into target
				drones[i]->setAlive(0);
				continue;
			}
			else */if (distance < drones[i]->getCommand()->info.surroundAndHover.outerRadius) {
				if (!drones[i]->getLocked()) { //if not locked, move until it's free to become locked
					bool blocked = false;
					for (unsigned int j = 0; j < drones.size(); j++) {
						if (!((drones[i]->getCommand()->id == drones[j]->getCommand()->id) && drones[j]->getLocked())) {
							continue;
						}
						double xDist2 = drones[j]->getXPos() - xPos;
						double yDist2 = drones[j]->getYPos() - yPos;
						double angle2 = atan2(yDist2, xDist2) * 180 / 3.14;
						//cout << angle << " " << angle2 << endl;
						if (fmod(abs(angle - angle2), 180) < 360 / drones.size() * 4) {
							blocked = true;
						}
					}
					if (!blocked) { //if the drone wasn't in the way of another locked drone, lock the drone in the current position
						drones[i]->setLocked(true);
					}
					else {
						circleOperation(drones[i], drones[i]->getCommand()->info.surroundAndHover.position.xPos,
							drones[i]->getCommand()->info.surroundAndHover.position.yPos,
							drones[i]->getCommand()->info.surroundAndHover.position.zPos, drones[i]->getCommand()->info.surroundAndHover.turnTime,
							drones[i]->getCommand()->info.surroundAndHover.counterclockwise);
					}
				}
				else {
					drones[i]->setXVel(0);
					drones[i]->setYVel(0);
					drones[i]->setZVel(0);
					drones[i]->setXAccel(0);
					drones[i]->setYAccel(0);
					drones[i]->setZAccel(0);
					continue;
				}
			}
			else if (distance < drones[i]->getCommand()->info.surroundAndHover.outerRadius * 1.5) {
				if (!drones[i]->getLocked()) { //if not locked, move until it's free to become locked
					bool blocked = false;
					for (unsigned int j = 0; j < drones.size(); j++) {
						if (!((drones[i]->getCommand()->id == drones[j]->getCommand()->id) && drones[j]->getLocked())) {
							continue;
						}
						double xDist2 = drones[j]->getXPos() - xPos;
						double yDist2 = drones[j]->getYPos() - yPos;
						double angle2 = atan2(yDist2, xDist2) * 180 / 3.14;
						if (fmod(abs(angle - angle2), 180) < 360 / drones.size() * 4) {
							blocked = true;
						}
					}
					if (!blocked) { //if the drone wasn't in the way of another locked drone, lock the drone in the current position
						drones[i]->setLocked(true);
					}
					else {
						targetOperation(drones[i], drones[i]->getCommand()->info.surroundAndHover.position.xPos,
							drones[i]->getCommand()->info.surroundAndHover.position.yPos,
							drones[i]->getCommand()->info.surroundAndHover.position.zPos);
					}
				}
				else {
					drones[i]->setXVel(0);
					drones[i]->setYVel(0);
					drones[i]->setZVel(0);
					drones[i]->setXAccel(0);
					drones[i]->setYAccel(0);
					drones[i]->setZAccel(0);
					continue;
				}
			}
			else {
				targetOperation(drones[i], drones[i]->getCommand()->info.surroundAndHover.position.xPos,
					drones[i]->getCommand()->info.surroundAndHover.position.yPos,
					drones[i]->getCommand()->info.surroundAndHover.position.zPos);
			}
		}
		else if (drones[i]->getCommand()->option == Command::SURROUND_AND_HOVER_AVOID_ANGLE) {
			double xPos = drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.xPos;
			double yPos = drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.yPos;
			double zPos = drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.zPos;
			double xDist = drones[i]->getXPos() - xPos;
			double yDist = drones[i]->getYPos() - yPos;
			double zDist = drones[i]->getZPos() - zPos;
			double distance = sqrt(xDist*xDist + yDist*yDist + zDist*zDist);
			double angle = atan2(yDist, xDist) * 180 / 3.14;

			//if (i == 75 || i == 69) cout << i << " " << angle << " " << distance << " " << drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.outerRadius << endl;
			//if (i == 79) cout << angle << endl;
			//cout << xDist << " " << yDist << " " << angle << endl;
			/*
			if (distance < drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.innerRadius) { //drone crashed into target
				drones[i]->setAlive(0);
				continue;
			}
			else */if (distance < drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.outerRadius) {
				if (!drones[i]->getLocked()) { //if not locked, move until it's free to become locked
					bool blocked = false;
					if (fmod(abs(angle - drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleToAvoid), 180) > drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleTolerance) {
						for (unsigned int j = 0; j < drones.size(); j++) {
							if (!((drones[i]->getCommand()->id == drones[j]->getCommand()->id) && drones[j]->getLocked())) {
								continue;
							}
							double xDist2 = drones[i]->getXPos() - drones[j]->getXPos();
							double yDist2 = drones[i]->getYPos() - drones[j]->getYPos();
							double angle2 = atan2(yDist2, xDist2) * 180 / 3.14;
							if (fmod(abs(angle - angle2), 180) < 360 / drones.size() * 4) {
								blocked = true;
							}
						}
					}
					else {
						blocked = true;
					}
					if (!blocked) { //if the drone wasn't in the way of another locked drone, lock the drone in the current position
						drones[i]->setLocked(true);
					}
					else {
						if (fmod(abs(angle - drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleToAvoid), 180) < drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleTolerance) {
							//if (i == 57) cout << angle << " " << drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleToAvoid << endl;
							if (angle > drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleToAvoid) {
								circleOperation(drones[i], drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.xPos,
									drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.yPos,
									drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.zPos,
									drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.turnTime, false);
							}
							else {
								circleOperation(drones[i], drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.xPos,
									drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.yPos,

									drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.zPos,
									drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.turnTime, true);
							}
						}
						else {
							circleOperation(drones[i], drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.xPos,
								drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.yPos,
								drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.zPos, 
								drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.turnTime, true);
						}
					}
				}
				else {
					drones[i]->setXVel(0);
					drones[i]->setYVel(0);
					drones[i]->setZVel(0);
					drones[i]->setXAccel(0);
					drones[i]->setYAccel(0);
					drones[i]->setZAccel(0);
					continue;
				}
			}
			else if (distance < drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.outerRadius * 2) {
				if (!drones[i]->getLocked()) { //if not locked, move until it's free to become locked
					bool blocked = false;
					if (fmod(abs(angle - drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleToAvoid), 180) > drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.angleTolerance) {
						for (unsigned int j = 0; j < drones.size(); j++) {
							if (!((drones[i]->getCommand()->id == drones[j]->getCommand()->id) && drones[j]->getLocked())) {
								continue;
							}
							double xDist2 = drones[i]->getXPos() - drones[j]->getXPos();
							double yDist2 = drones[i]->getYPos() - drones[j]->getYPos();
							double angle2 = atan2(yDist2, xDist2) * 180 / 3.14;
							if (fmod(abs(angle - angle2), 180) < 360 / drones.size() * 4) {
								blocked = true;
							}
						}
					}
					else {
						blocked = true;
					}
					if (!blocked) { //if the drone wasn't in the way of another locked drone, lock the drone in the current position
						drones[i]->setLocked(true);
					}
					else {
						targetOperation(drones[i], drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.xPos,
							drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.yPos,
							drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.zPos);
					}
				}
				else {
					drones[i]->setXVel(0);
					drones[i]->setYVel(0);
					drones[i]->setZVel(0);
					drones[i]->setXAccel(0);
					drones[i]->setYAccel(0);
					drones[i]->setZAccel(0);
					continue;
				}
			}
			else {
				targetOperation(drones[i], drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.xPos,
					drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.yPos,
					drones[i]->getCommand()->info.surroundAndHoverAvoidAngle.position.zPos);
			}
		}
		else {
			drones[i]->setXAccel(0);
			drones[i]->setYAccel(0);
			drones[i]->setZAccel(0);
		}

		for (int j = i + 1; j < NUM_DRONES; j++) {
			double xDist = drones[i]->getXPos() - drones[j]->getXPos();
			double yDist = drones[i]->getYPos() - drones[j]->getYPos();
			double zDist = drones[i]->getZPos() - drones[j]->getZPos();
			double distance = sqrt(xDist*xDist + yDist*yDist + zDist*zDist);
			if (distance < 2) {
				drones[i]->setAlive(0);
				drones[j]->setAlive(0);
				cout << "Drone " << i << " crashed into drone " << j << " :(" << endl;
			} else if (distance < 50 && 
				(abs(drones[j]->getXVel() * drones[j]->getYVel()) > 0.1*(drones[j]->getMaxXYVel()*drones[j]->getMaxXYVel()))) {
				drones[i]->setZAccel(drones[i]->getMaxZAccel());
				//cout << distance << " "<< drones[i]->getCommand()->option << " " << drones[i]->getXVel() << " " << drones[i]->getYVel() << endl;
				break;
			}
		}
		//cout << i << endl;
	}
}

//optimally assigns the commands in COMMANDQUEUE to all the drones in DRONES
void World::assignCommands() {
	if (commandsAssigned == true) { //no changes needed to be made in the command distribution
		return;
	}
	
	//If one of the active commands is NONE, deactivates all commands that occurred prior to that NONE
	//If one of the active commands is X/Y/Z-ACCEL, assigns all the drones to the highest priority command of that type
	int prioritySum = 0;
	int allDroneCommandIndex = -1;
	unsigned int allDroneCommandPriority = 0;
	for (unsigned int i = 0; i < commands.size(); i++) {
		if (!commands[i].activeCommand) {
			continue;
		}

		prioritySum += commands[i].priority;

		switch (commands[i].option) {
		case Command::NONE:
			for (unsigned int j = 0; j <= i; j++) {
				commands[j].activeCommand = false;
			}
			assignCommands(); //renew the assignment process with this new deactivation in mind
			break;
		case Command::X_ACCEL:
		case Command::Y_ACCEL:
		case Command::Z_ACCEL: 
		case Command::RETURN_TO_BASE:
			if (commands[i].priority >= allDroneCommandPriority) {
				allDroneCommandIndex = i;
				allDroneCommandPriority = commands[i].priority;
			}
			break;
		}
	}

	if (allDroneCommandIndex != -1) {
		for (unsigned int i = 0; i < drones.size(); i++) {
			drones[i]->setCommand(commands[allDroneCommandIndex]);

			//cout << i << " " << drones[i]->getZPos() << endl;
		}
		commandsAssigned = true;
		return;
	}

	//If there are no commands that automatically take up all the drones, divide the rest of the tasks by priority
	//Divide the drones to the target that minimize their distance_to_target/task_priority ratio
	//Ex. If drone1 is 500 meters from target 1 (priority 2) and 600 meters from target 2 (priority 3), it would choose
	//	target 2 (because 600/3 = 200 < 500/2 = 250)
	#pragma omp parallel for
	for (unsigned int i = 0; i < drones.size(); i++) {
		if (!drones[i]->getAlive()) {
			continue;
		}

		double minDPRatio = std::numeric_limits<double>::max(); //distance-to-priority ratio
		int minTaskIndex = -1;
		for (unsigned int j = 0; j < commands.size(); j++) {
			if (!commands[j].activeCommand) {
				continue;
			}

			double xDist = commands[j].info.position.xPos - drones[i]->getXPos();
			double yDist = commands[j].info.position.yPos - drones[i]->getYPos();
			double zDist = commands[j].info.position.zPos - drones[i]->getZPos();
			double distance = sqrt(xDist*xDist + yDist*yDist + zDist*zDist);
			
			if (commands[j].option == Command::CIRCLE) {
				distance = abs(distance - commands[j].info.turn.radius);
			}
			double DPRatio = distance / commands[j].priority;
			if (DPRatio < minDPRatio) {
				minDPRatio = DPRatio;
				minTaskIndex = j;
			}
		}
		if (minTaskIndex != -1) { //no active commands, shouldn't happen
			drones[i]->setCommand(commands[minTaskIndex]);
		}
	}

	commandsAssigned = true;
}

void World::targetOperation(Drone *d, double xPos, double yPos, double zPos) {
	double xDist = xPos - d->getXPos();
	double yDist = yPos - d->getYPos();
	double zDist = zPos- d->getZPos();
	double xyDistance = sqrt(xDist*xDist + yDist*yDist);
	double distance = sqrt(xyDistance*xyDistance + zDist*zDist);
	d->setXAccel(d->getMaxXYAccel() * xDist / xyDistance / 2.0);
	d->setYAccel(d->getMaxXYAccel() * yDist / xyDistance / 2.0);
	d->setZAccel(d->getMaxZAccel() * zDist / distance / 2.0);

	//if the current command is to return to base and the drone reaches base, deletes the current drone and adds a new one
	if (d->getCommand()->option == Command::RETURN_TO_BASE && distance < 100) {
		d->reset();
	}
}

void World::circleOperation(Drone *d, double xPos, double yPos, double zPos, double turnTime, bool counterclockwise) {
	double xDist = xPos - d->getXPos();
	double yDist = yPos - d->getYPos();
	double zDist = zPos - d->getZPos();
	double xyDistance = sqrt(xDist*xDist + yDist*yDist);
	double distance = sqrt(xyDistance*xyDistance + zDist*zDist);
	double v = 2 * 3.14*xyDistance / turnTime;
	int ccw = counterclockwise ? 1 : -1;
	d->setXVel(v*yDist / xyDistance*ccw);
	d->setYVel(-v*xDist / xyDistance*ccw);
	d->setXAccel(0);
	d->setYAccel(0);
	d->setZAccel(d->getMaxZAccel() * zDist / distance / 2.0);
}



/**
TODO: Fully implement effectiveXAccel() and effectiveYAccel() to factor wind speeds
	  Once implemented, uncomment the lines xAccel and yAccel in update()
**/
double World::effectiveXAccel(double xAccel, double MAX_XY_ACCEL) {
	switch (windHeading) {
	case EAST: return (xAccel + windSpeed > MAX_XY_ACCEL) ? MAX_XY_ACCEL - windSpeed : xAccel; break;
	case WEST: return xAccel - windSpeed; break;
	case NORTHEAST:
	case SOUTHEAST: return (xAccel + 0.707*windSpeed > MAX_XY_ACCEL) ? MAX_XY_ACCEL - 0.707*windSpeed : xAccel; break;
	case NORTHWEST:
	case SOUTHWEST: return xAccel - 0.707*windSpeed; break;
	default: return xAccel;
	}
}

double World::effectiveYAccel(double yAccel, double MAX_XY_ACCEL) {
	switch (windHeading) {
	case NORTH: return (yAccel + windSpeed > MAX_XY_ACCEL) ? MAX_XY_ACCEL - windSpeed : yAccel; break;
	case SOUTH: return yAccel - windSpeed; break;
	case NORTHEAST:
	case NORTHWEST: return (yAccel + 0.707*windSpeed > MAX_XY_ACCEL) ? MAX_XY_ACCEL - 0.707*windSpeed : yAccel; break;
	case SOUTHEAST:
	case SOUTHWEST: return yAccel - 0.707*windSpeed; break;
	default: return yAccel;
	}
}