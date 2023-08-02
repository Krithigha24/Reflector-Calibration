# Reflector-Calibration
The main objective of the project is to accurately estimate the distance of cylindrical reflectors from a Lidar using Laser Scan messages. The project utilizes a cylindrical reflector with a radius of 0.045m and a Hokuyo UST-30LX Lidar. 

## Method to Estimate Distance of Cylindrical Reflector from Lidar
![Image demonstrating laser beams hitting reflector](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRsGi0FenxVxsT4jgPwIKAaD3yKX6Ivwn5qCtYlRmWoY_7FPJnQskI-BX_VRu1E6n9YAA&usqp=CAU)

1. Calculate the distance from the Lidar (assumed to be at (0,0) coordinates) to each data point belonging to the reflector in a single laser scan.
2. Select the two closest data points to the Lidar.
3. Take the average of the two selected data points' coordinates to approximate the position where the laser beam is normal to the cylindrical reflector's surface.
4. Extend the line of sight from the incident normal beam by the known radius amount to locate the center coordinates of the reflector and thereby determine the distance from the Lidar.

This logic is coded in processBag.py under scripts folder 

## Collecting Data
The cylindrical reflector is positioned at intervals of 0.05m, ranging from 0.129m to 3m away from the Lidar. At each distance experiment, a Laser Scan message is recorded.

Connect Hokuyo UST-30LX Lidar to laptop and bringup hokuyo node + rviz
```
roslaunch calibration collectData.launch
```

Navigate to test_bags folder and rosbag record laser scan message after setting up the reflector at a distance
Record for 30 seconds and naming convention of bag should follow as exp_[actual measured distance to lidar in mm].bag 
```
timeout 30 rosbag record -O exp_2620.bag /scan
```

## Binary Segmentation and executing method to estimate distance to lidar
