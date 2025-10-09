# Atta-Bot Project

## Overview
The Atta-Bot project is designed to control a robotic system that can communicate with other robots and navigate autonomously. The project consists of an Arduino sketch and utility files that manage the robot's operations, including movement, obstacle detection, and sensor readings.

## Project Structure
```
Atta-Bot-P_ed
├── Controller
│   └── AttaBot
│       ├── AttaBot.ino
│       ├── utils.h
│       └── utils.cpp
├── README.md
```

## Files Description

### `Controller/AttaBot/AttaBot.ino`
This file contains the main Arduino sketch for the robot. It includes:
- The `setup()` function to initialize the robot's components.
- The `loop()` function that manages the robot's state machine.
- Functions for movement, obstacle detection, and sensor reading.
- Communication handling with other robots.

### `Controller/AttaBot/utils.h`
This header file declares utility functions and constants used throughout the project. It includes:
- Definitions for data structures.
- Function prototypes for helper functions.
- Constants related to the robot's operation.

### `Controller/AttaBot/utils.cpp`
This file implements the utility functions declared in `utils.h`. It contains:
- Logic for various helper functions that support the main functionality of the robot.

## Usage Instructions
1. **Setup**: Connect the robot's components as specified in the `AttaBot.ino` file. Ensure that the necessary libraries are installed in your Arduino IDE.
2. **Upload**: Upload the `AttaBot.ino` sketch to your Arduino board.
3. **Communication**: The robot will automatically connect to the specified Wi-Fi network and listen for commands from other robots.
4. **Movement Command**: To send a command for all robots to move to a specified location, use the format `POSITIONGT|x|y`, where `x` and `y` are the target coordinates.

## Features
- Autonomous navigation and obstacle avoidance.
- Communication with multiple robots.
- Ability to move to specified coordinates based on received commands.
- Real-time sensor data processing.

## Future Enhancements
- Implement advanced pathfinding algorithms.
- Improve obstacle detection capabilities.
- Enhance communication protocols for better reliability.