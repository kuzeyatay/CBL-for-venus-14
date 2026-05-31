# 5EID0 Engineering challenge for Venus - Group 14
## Overview
The goal of the system is to explore an unknown terrain using two autonomous robots based on the provided PYNQ-Z2 board. The robots traverse a planet like map, while avoiding obstacles, classifying rock samples, and relaying information to the base station. The base station must visualize the relative positions of cliffs, hills, boundaries, and rock samples on a graphical map. 
## Project Architecture
* `chassis/`: Movement Logic
  * `drive/`: Motor control
  * `navigation/`: Path planning and obstacle avoidance algorithms
  * `odom/`: Odemetry calculations for tracking position over time

* `comms/` Communication protocol
* `constants/` Global configuration settings and types
* `gui/`  Visualization and user interface
  * `src/` GUI Javascript source code
* `sensors/` Environmental sensors
  * `color/` `distance/` `temperature/` Drivers for collecting environmental data
## Build and Setup
### Prerequisites
### Configuration
### Compilation
