# Drone-Swarm-Simulation

This application works to simulate drone swarm behavior in response to common group maneuver patterns. It uses a C++ backend to parallelize drone movement calculations across multiple threads, as well as a Java frontend for visualization. For reference, the C++ backend is executed by DroneSwarmSimulation.sln and drawing from the files in the DroneSwarmSimulation folder, and the Java code is contained in the DroneSwarmSimulation/DroneSwarmGUI folder.

The currently supported movement patterns are:
 •	Circling/hovering around a specified target within a certain minimum and maximum range
 •	Doing the above while maintaining a specified angular range of movement (for example, only circling between 0 degrees and 180 degrees when viewed from the top down)
 •	Returning to a central base location when battery is low for repair (battery consumption is calculated in Drone.cpp if you're curious about the details)

In addition, all movement is combined with obstacle avoidance and compensation for external wind speed. Currently, movement is calculated using a proportional control model, but in the future a more comprehensive PID or alternative control model will be looked into (see World.cpp if you're curious about the details).

Tasks are given by the Source.cpp function in the DroneSwarmSimulation folder, simulating a central base station that assigned multiple tasks of varying priorities to the drone. In the nature of swarm robotics, each drone autonomously decides which task to execute based on its surroundings, analyzing its distance to the target locations as well as the respective priority of each task to optimize how many drones are assigned to each task (see World.cpp and Drone.cpp if you're curious about the details).
