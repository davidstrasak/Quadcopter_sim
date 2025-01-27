# Quadcopter Simulator

A MATLAB-based quadcopter simulator developed as part of the "Intelligent Control Systems" course. This simulator implements a full state observer for control and features waypoint tracking with safety checks.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Technical Details](#technical-details)
- [Contributors](#contributors)

## Overview
This project simulates a quadcopter's flight through a series of waypoints using advanced control techniques. The simulator includes:
- Full state observer for robust state estimation
- Waypoint tracking with time constraints
- Safety checks for crash detection
- 3D visualization of the quadcopter's movement

## Features
- **Full State Observer**: Estimates all system states for precise control
- **Waypoint Tracking**: Follows predefined waypoints with specified timing
- **Safety Checks**: Monitors for potential crashes and waypoint validation
- **3D Visualization**: Real-time visualization of the quadcopter's position and orientation
- **Control Architecture**: Implements separate controllers for altitude, position, and attitude

## Requirements
- MATLAB
- Control Systems Toolbox
- Robotics Systems Toolbox (optional for visualization)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/quadcopter-simulator.git
   ```
2. Open MATLAB and navigate to the project directory
3. Run the main simulation script:
   ```matlab
   quadcopter_simulator
   ```

## Usage
The simulator is controlled through a main script that sets up the environment and runs the simulation. Key parameters that can be adjusted:
- wayPoints: Define the trajectory the quadcopter should follow
- timeForWaypointPassage: Specify the exact times when the quadcopter should reach each waypoint
- positionTolerance: Set the acceptable error margin for waypoint tracking
- deltaT: Simulation time step
- simulationTime: Total duration of the simulation

## Technical Details
### Control Architecture
The quadcopter uses a cascaded control approach:
1. **Altitude Control**: Regulates the vertical position using a full state observer
2. **Position Control**: Controls movement in both longitudinal (X-axis) and lateral (Y-axis) directions
3. **Attitude Control**: Maintains the desired roll and pitch angles using angular rate feedback

### State Estimation
A full state observer is implemented to estimate:
- Position (X, Y, Z)
- Velocity (X, Y, Z)
- Euler angles (Roll, Pitch, Yaw)
- Angular rates (p, q, r)

### Safety Features
- **Crash Detection**: Monitors the quadcopter's altitude and triggers an emergency stop if it crashes
- **Waypoint Validation**: Verifies that the quadcopter passes through each waypoint within the specified tolerance

## Contributors
- David Strasak - Wrote the regulation, visualization and safety features.
- Jiri Kovar - Initial instructions and code snippets.

## License
My professor made this and it's only for Intelligent Control Systems students
