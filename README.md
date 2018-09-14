# TankBot

TankBot is a small chain-driven robot that I am building as a way to improve my understanding of robotic algorithms.

## Hardware

  - Chassis: fully metal chassis bought in China (TS100)， around 50 USD, expanded with two levels
  - Two brushed 12V motors that drive the left and right belt
  - Motor controller: DVR8833 Mosfet-based, 2x1.5A continuous (sufficient for the motors)
  - 12V battery (currently 3S 18650 Li-Ion cells)
  - Main processing board: Raspberry Pi 3 (planned to be upgraded to something more powerful at some point)
  - Sensor/motor controller: Arduino Mega (perhaps a Uno is sufficient, but there was enough space)
  - Sensors: YDLidar 2D lidar, Raspberry camera (front facing), IMU sensor

## Software
- Planned features: lane and object recognition through camera, lidar data processing with PCL, SLAM

## Installation

### Requirements
- PCL (Point Cloud Library) >= 1.7 - http://pointclouds.org/downloads/
- Cmake >= 3.1
- Linux (recommended Ubuntu >= 16.04)
- Potentially additional requirements for ydlidar (see https://github.com/EAIBOT/ydlidar/tree/master/sdk)

### Installation
```sh
$ git clone https://github.com/jhallier/TankBot
$ cd TankBot
$ mkdir build
$ cd build
$ cmake ..
$ make
$./tankbot
```

## Photos
![TankBot Overview](TankBot.jpg "Overview of TankBot")