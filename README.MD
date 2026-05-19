from pathlib import Path

readme = """# Robotic Arm Square Drawing Project

A 3DOF robotic arm project built with **MATLAB** and **Arduino Uno** to simulate and control a servo-based robot arm that can draw a square trajectory in 3D space.

## Project Overview

This project models a robotic arm using Denavit–Hartenberg (DH) parameters, solves inverse kinematics (IK) for a square path, and sends joint commands to three MG996R servo motors through an Arduino Uno.

The system includes:

- MATLAB rigid body robot modeling
- Inverse kinematics for trajectory following
- Real-time servo actuation through Arduino
- 3D visualization of the robot arm motion
- Reachability checking before motion execution

## Team Members

- Veronica Abdelnour
- Raneen Mohamed
- Lojyn Tarek

## Supervisor

- Dr. Mostafa Fouz

## Features

- Builds a robotic arm model using `rigidBodyTree`
- Uses standard DH parameters for kinematic description
- Solves IK for target points along a square path
- Checks whether all target points are reachable before motion starts
- Converts joint angles from radians to servo-safe degrees
- Controls 3 servo motors on an Arduino Uno
- Displays the arm motion in MATLAB with a live 3D plot

## Repository Contents

- `main.c` — MATLAB control script for robot modeling, IK, visualization, and servo control
- `preview.png` — preview image of the project
- `robotics.....................file.docx` — project report/documentation

> Note: the main control file contains MATLAB code. For MATLAB execution, it is recommended to rename it from `main.c` to `main.m`.

## Hardware Used

- **Arduino Uno** — main microcontroller
- **3 × MG996R servo motors** — base, shoulder, and elbow joints
- **Buck converter** — steps 12V down to 6V for the servos
- **12V power supply** — powers the system through the buck converter
- **External wiring and common ground** — ensures stable servo control

## Servo Pin Mapping

- Servo 1 → `D3`
- Servo 2 → `D4`
- Servo 3 → `D5`

## Kinematic Model

The robot is modeled using the following DH parameters:

| Link | a | α | d | θ |
|------|---|---|---|---|
| 1 | 0 | 0 | `L0` | 0 |
| 2 | `Lb1` | π/2 | `La1` | 0 |
| 3 | `L2` | -π | 0 | 0 |
| 4 | `L3` | 0 | 0 | 0 |

### Link Dimensions

- `L0 = 0.07 m`
- `La1 = 0.10 m`
- `Lb1 = 0.01 m`
- `L2 = 0.12 m`
- `L3 = 0.21 m`

## Motion Task

The end-effector follows a square trajectory:

`P1 → P2 → P3 → P4 → P1`

With corner points defined as:

- `P1 = [X, Ystart, Zstart]`
- `P2 = [X + S, Ystart, Zstart]`
- `P3 = [X + S, Ystart + S, Zstart - S]`
- `P4 = [X, Ystart + S, Zstart - S]`

Where:

- `S = 0.04 m`

## Software Workflow

1. Connect MATLAB to the Arduino Uno.
2. Create the robot model using DH parameters.
3. Initialize the robot in its home configuration.
4. Generate a square trajectory with interpolation.
5. Check reachability for every corner point.
6. Solve inverse kinematics for every point along the path.
7. Convert angles to servo input range.
8. Send commands to the servos.
9. Update the live robot visualization.

## How It Works

### 1. Robot Modeling
The robot is defined with `rigidBodyTree`, rigid bodies, and revolute joints. DH transforms are applied to each joint to match the physical structure.

### 2. Reachability Check
Before movement begins, each corner of the square is checked using inverse kinematics. If any point is unreachable, the trajectory is canceled.

### 3. Trajectory Generation
MATLAB’s `transformtraj` is used to create smooth motion between the four corners of the square.

### 4. Servo Actuation
Joint angles are converted from radians to degrees, clipped to safe ranges, then normalized to the servo input range before sending to Arduino using `writePosition()`.

## MATLAB Requirements

You need:

- MATLAB
- Robotics System Toolbox
- Arduino support in MATLAB
- Servo library support

## Arduino Requirements

- Arduino Uno
- Servo motors connected to pins `D3`, `D4`, and `D5`
- External 6V servo supply through a buck converter
- Common ground between Arduino and external supply

## Usage

1. Open the MATLAB script.
2. Make sure the Arduino port is correct:
   ```matlab
   a = arduino('COM5','Uno','Libraries','Servo');
