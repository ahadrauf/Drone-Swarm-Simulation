#include <iostream>
#include <string>
#include <fstream>
#include <omp.h>
using namespace std;

#include "World.h"
#include "Drone.h"

#define FIB_FACTOR 1000

void updateCommands(World *world, int turn);
string progressBar(int curr, int max);

int main() {
	//Define world and drones
	World world(2000*FIB_FACTOR, 2000*FIB_FACTOR, 800, 5, NORTH, 0.1);
	const int NUM_DRONES = 1000;
	const int NUM_SIMULATION_CYCLES = 1000; //5,000 cycles * 100ms = 500 seconds = 8 minutes
	Drone *drones[NUM_DRONES];
	for (int i = 0; i < NUM_DRONES; i++) {
		//Put 1, 2, 3, ..., 10 in the first row
		//Put 11, 12, 13, ..., 20 in the second row
		//Continue for the rest of the rows
		drones[i] = new Drone(50 * FIB_FACTOR * (i % 10 - 5), 50 * FIB_FACTOR * (i / 10 - 5));
		world.addDrone(drones[i]);
	}

	//Write header to output file to define world
	ofstream out("data.txt");
	out << (world.getXLength() / FIB_FACTOR) << " " << (world.getYLength() / FIB_FACTOR) << " " << world.getZLength() << " " << 
		world.getWindSpeed() << " " << world.getWindHeading() << " " << world.getT() << " " << NUM_DRONES << " " << NUM_SIMULATION_CYCLES << endl;

	//Begin simulation
	for (int i = 0; i < NUM_SIMULATION_CYCLES; i++) {
		if (i == 0 || (i + 1) % (NUM_SIMULATION_CYCLES / 10) == 0) {
			//system("cls");
			cout << "Simulation Progress: " << progressBar(i + 1, NUM_SIMULATION_CYCLES) << endl;
		}
		updateCommands(&world, i);
		world.update();
		/*
		for (int j = 0; j < 10; j++) {
			cout << i << " " << j << " " << drones[j]->getZAccel() << " " << drones[j]->getZPos() << endl;
		}
		//*/
		//Write output data
		vector<Command> commands = world.getCommands();
		int numCommands = 0;
		for (unsigned int j = 0; j < commands.size(); j++) {
			if (!commands[j].activeCommand) {
				continue;
			}
			switch (commands[j].option) {
			case Command::TARGET:
			case Command::CIRCLE:
			case Command::SURROUND_AND_CIRCLE:
			case Command::SURROUND_AND_HOVER: 
			case Command::SURROUND_AND_HOVER_AVOID_ANGLE: numCommands++; break;
			}
		}
		out << numCommands << endl;
		for (unsigned int j = 0; j < commands.size(); j++) {
			if (!commands[j].activeCommand) {
				continue;
			}
			switch (commands[j].option) {
			case Command::TARGET: out << commands[j].info.position.xPos / FIB_FACTOR << " " << commands[j].info.position.yPos / FIB_FACTOR << 
										" " << commands[j].info.position.zPos << " " << 1000 / FIB_FACTOR << endl; break;
			case Command::CIRCLE: out << commands[j].info.turn.position.xPos / FIB_FACTOR << " " << 
										commands[j].info.turn.position.yPos / FIB_FACTOR << " " << 
										commands[j].info.turn.position.zPos << " " << 
										commands[j].info.turn.radius / FIB_FACTOR << endl; break;
			case Command::SURROUND_AND_CIRCLE: out << commands[j].info.surroundAndCircle.position.xPos / FIB_FACTOR << " " << 
										commands[j].info.surroundAndCircle.position.yPos / FIB_FACTOR << " " << 
										commands[j].info.surroundAndCircle.position.zPos << " " << 
										commands[j].info.surroundAndCircle.innerRadius / FIB_FACTOR << endl; break;
			case Command::SURROUND_AND_HOVER: out << commands[j].info.surroundAndHover.position.xPos / FIB_FACTOR << " " << 
										commands[j].info.surroundAndHover.position.yPos / FIB_FACTOR << " " << 
										commands[j].info.surroundAndHover.position.zPos << " " << 
										commands[j].info.surroundAndHover.innerRadius / FIB_FACTOR << endl; break;
			case Command::SURROUND_AND_HOVER_AVOID_ANGLE: out << commands[j].info.surroundAndHoverAvoidAngle.position.xPos / FIB_FACTOR << " " <<
				commands[j].info.surroundAndHoverAvoidAngle.position.yPos / FIB_FACTOR << " " <<
				commands[j].info.surroundAndHoverAvoidAngle.position.zPos << " " <<
				commands[j].info.surroundAndHoverAvoidAngle.innerRadius / FIB_FACTOR << endl; break;
			}
		}
		for (int j = 0; j < NUM_DRONES; j++) {
			out << /*i << " " << j << " " <<*/ (drones[j]->getXPos() / FIB_FACTOR) << " " << (drones[j]->getYPos() / FIB_FACTOR) << " " << drones[j]->getZPos()/* << " " << drones[j]->getXVel() << " " << drones[j]->getYVel() << " " << drones[j]->getZVel() << " " << drones[j]->getXAccel() << " " << drones[j]->getYAccel() << " " << drones[j]->getZAccel() << " " << drones[j]->getBatteryLife()*/ << " " << drones[j]->getAlive() << endl;
		}
	}

	out.close();
	for (int i = 0; i < NUM_DRONES; i++) { //free the memory
		delete(drones[i]);
	}
	return 0;
}

void updateCommands(World *world, int turn) {
	if (turn == 0) {
		Command c = { 0, Command::Z_ACCEL, 1 };
		c.info.value = 5;
		world->sendCommand(c);
	}
	else if (turn == 200) {
		world->stop();
		Command c = { 1, Command::Z_ACCEL, 1 };
		c.info.value = -5;
		world->sendCommand(c);
	}
	else if (turn == 250) {
		world->stop();
		/*
		Command c1 = { Command::SURROUND_AND_HOVER, 10 };
		c1.info.surroundAndHover = { { 500 * FIB_FACTOR, 500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 100, true, false };
		world->sendCommand(c1);

		Command c2 = { Command::SURROUND_AND_HOVER, 10 };
		c2.info.surroundAndHover = { { -500 * FIB_FACTOR, 500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 100, true, false };
		world->sendCommand(c2);

		Command c3 = { Command::SURROUND_AND_HOVER, 10 };
		c3.info.surroundAndHover = { { 500 * FIB_FACTOR, -500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 100, true, false };
		world->sendCommand(c3);

		Command c4 = { Command::SURROUND_AND_HOVER, 10 };
		c4.info.surroundAndHover = { { -500 * FIB_FACTOR, -500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 100, false, false };
		world->sendCommand(c4);
		*/
		///*
		Command c1 = { 2, Command::SURROUND_AND_HOVER, 10 };
		c1.info.surroundAndHover = { { 500 * FIB_FACTOR, 500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 150 * FIB_FACTOR, 100, true };
		world->sendCommand(c1);

		Command c2 = { 3, Command::SURROUND_AND_HOVER, 10 };
		c2.info.surroundAndHover = { { -500 * FIB_FACTOR, 500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 150 * FIB_FACTOR, 100, true };
		world->sendCommand(c2);

		Command c3 = { 4, Command::SURROUND_AND_HOVER, 10 };
		c3.info.surroundAndHover = { { 500 * FIB_FACTOR, -500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 150 * FIB_FACTOR, 100, true };
		world->sendCommand(c3);

		Command c4 = { 5, Command::SURROUND_AND_HOVER, 14 };
		c4.info.surroundAndHover = { { -700 * FIB_FACTOR, -700 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 150 * FIB_FACTOR, 100, false };
		world->sendCommand(c4);
		//*/
		/*
		Command c1 = { Command::TARGET, 30 };
		c1.info.position = { 500, 500, 500 };
		world->sendCommand(c1);

		Command c2 = { Command::TARGET, 10 };
		c2.info.position = { -500, 500, 500 };
		world->sendCommand(c2);

		Command c3 = { Command::TARGET, 10 };
		c3.info.position = { 500, -500, 500 };
		world->sendCommand(c3);

		Command c4 = { Command::TARGET, 10 };
		c4.info.position = { -500, -500, 500 };
		world->sendCommand(c4);

		Command c5 = { Command::CIRCLE, 1 };
		c5.info.turn = { { 0, 0, 500 }, 500, 100, true };
		world->sendCommand(c5);
		*/
	}
	else if (turn == 3000) {
		///*
		Command c1 = { 2, Command::SURROUND_AND_HOVER_AVOID_ANGLE, 10 };
		c1.info.surroundAndHoverAvoidAngle = { { 500 * FIB_FACTOR, 500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 50, true, -40, 45 };
		world->updateCommand(2, c1);

		Command c2 = { 3, Command::SURROUND_AND_HOVER_AVOID_ANGLE, 10 };
		c2.info.surroundAndHoverAvoidAngle = { { -500 * FIB_FACTOR, 500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 50, true, -30, 45 };
		world->updateCommand(3, c2);

		Command c3 = { 4, Command::SURROUND_AND_HOVER_AVOID_ANGLE, 10 };
		c3.info.surroundAndHoverAvoidAngle = { { 500 * FIB_FACTOR, -500 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 50, true, 60, 45 };
		world->updateCommand(4, c3);

		Command c4 = { 5, Command::SURROUND_AND_HOVER_AVOID_ANGLE, 14 };
		c4.info.surroundAndHoverAvoidAngle = { { -700 * FIB_FACTOR, -700 * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 50, false, 30, 45 };
		world->updateCommand(5, c4);
		//*/
	}
	else if (turn > 4000 && turn < 5000) {
		Command c4 = { 5, Command::SURROUND_AND_HOVER_AVOID_ANGLE, 30 };
		c4.info.surroundAndHoverAvoidAngle = { { (-700 + (turn - 4000) / 5) * FIB_FACTOR, (-700 + (turn - 4000) / 5) * FIB_FACTOR, 500 }, 50 * FIB_FACTOR, 100 * FIB_FACTOR, 50, false, 30, 45 };
		world->updateCommand(5, c4);
	}
	else if (turn == 6500) {
		world->stop();
		Command c4 = { 6, Command::RETURN_TO_BASE, 30 };
		world->sendCommand(c4);
	}
	/*
	#pragma omp parallel for
	for (int j = 0; j < NUM_DRONES; j++) {
		if (i < 100) { //spend 100*0.1 = 10 seconds flying up to 250 meters
			commands[j].option = Command::Z_ACCEL;
			commands[j].info.value = 5;
		}
		else if (i < 200) { //spend the next 10 seconds to zero out z velocity (end up at 500 meters)
			commands[j].option = Command::Z_ACCEL;
			commands[j].info.value = -5;
		}
		else { //for the rest of the time, divide the drones into four groups that each pursue a target
			commands[j].option = Command::TARGET;
			if (j < NUM_DRONES / 5) {
				commands[j].info.position.xPos = 500;
				commands[j].info.position.yPos = 500;
				commands[j].info.position.zPos = 500;
			}
			else if (j < 2 * NUM_DRONES / 5) {
				commands[j].info.position.xPos = -500;
				commands[j].info.position.yPos = 500;
				commands[j].info.position.zPos = 500;
			}
			else if (j < 3 * NUM_DRONES / 5) {
				commands[j].info.position.xPos = -500;
				commands[j].info.position.yPos = -500;
				commands[j].info.position.zPos = 500;
			}
			else if (j < 4 * NUM_DRONES / 5) {
				commands[j].info.position.xPos = 500;
				commands[j].info.position.yPos = -500;
				commands[j].info.position.zPos = 500;
			}
			else {
				if (i < 1500) {
					commands[j].option = Command::TARGET;
					commands[j].info.position = { -500, 0, 500 };
				} else {
					commands[j].option = Command::CIRCLE;
					commands[j].info.turn = { 0, 0, 500, 100, true };
				}
			}
		}
	}
	*/
}

string progressBar(int curr, int max) {
	int i = 0;
	int currPlace = curr / (max / 10);
	string ret = "";
	while (i < currPlace && i < 10) {
		ret += "#";
		i++;
	}
	while (i < 10) {
		ret += "-";
		i++;
	}
	return "|" + ret + "|";
}