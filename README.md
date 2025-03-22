# MazeSolver Project

## Hardware Requirements

1. Arduino UNO board
2. 3 Ultrasonic sensors (HC-SR04)
3. 2 DC motors with wheels
4. Motor driver (L298N)
5. Chassis for the robot
6. Power supply (Battery)

## Pin Connections

### Ultrasonic Sensors
- Left Sensor: Trigger=A4, Echo=A5
- Front Sensor: Trigger=A2, Echo=A3 
- Right Sensor: Trigger=A0, Echo=A1

### Motors
- Left Motor: Forward=6, Backward=3, Enable=9
- Right Motor: Forward=5, Backward=10, Enable=11

## Key Parameters to Adjust

### PID Controller Settings
```cpp
double Kp = 1;     // Proportional gain
double Ki = 0.05;  // Integral gain 
double Kd = 0.01;  // Derivative gain
```
These values control how well the robot stays centered in the maze path. Adjust if robot wobbles or hits walls.

### Turn Timing Parameters
```cpp
int LturnTime = 500;   // Left turn duration
int RturnTime = 560;   // Right turn duration
int UturnTime = 1050;  // U-turn duration
int FATTime = 575;     // Forward after turn time
```
These control how long each turn movement lasts. Adjust based on your robot's speed and maze dimensions.

### Distance Thresholds
```cpp
double threshold = 0;  // Initial side wall detection threshold
double fThreshold = 0; // Initial front wall detection threshold
```
The thresholds are dynamically calculated when the robot starts, based on the maze dimensions:
- `fThreshold = leftDistance + rightDistance + 10` (corridor width plus margin)
- `threshold = mean * 4` (where mean is average of left and right distances)

This auto-calibration ensures the robot adapts to different maze sizes. The thresholds determine when the robot decides to turn.

## Setup Instructions

1. Hardware Assembly:
   - Mount the 3 ultrasonic sensors (left, front, right)
   - Connect motors to motor driver
   - Wire everything to Arduino according to pin definitions

2. Software Setup:
   1. Install Arduino IDE 
   2. Install CH34X driver for Arduino communication
   3. Install PID Library: https://github.com/br3ttb/Arduino-PID-Library
   4. Upload the code to Arduino

3. Calibration Steps:
   - Start with default PID values
   - Test robot in a straight corridor
   - If robot wobbles: decrease Kp
   - If robot hits walls: increase Kp
   - Adjust turn timings until robot makes proper 90° turns
   - Fine-tune threshold values based on maze dimensions

## How It Works

The robot uses a Right Wall Following Algorithm with PID control:
1. Continuously reads distances from all 3 sensors
2. Uses PID to maintain equal distance from side walls
3. Makes decisions based on sensor readings:
   - Right wall open -> Turn right
   - Front wall clear -> Go straight
   - Left wall open -> Turn left
   - Dead end -> U-turn

The code in [Code.cpp](d:\Documents\MazeSolver\Code.cpp) contains all the logic for navigation and motor control.

Remember to test thoroughly in your specific maze as dimensions and surface conditions can affect performance.

