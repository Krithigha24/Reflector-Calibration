# Reflector-Calibration
The main objective of the project is to accurately estimate the distance of cylindrical reflectors from a Lidar using Laser Scan messages. The project utilizes a cylindrical reflector with a radius of 0.045m and a Hokuyo UST-30LX Lidar. 
### Method to Estimate Distance of Cylindrical Reflector from Lidar
![Image demonstrating laser beams hitting reflector](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRsGi0FenxVxsT4jgPwIKAaD3yKX6Ivwn5qCtYlRmWoY_7FPJnQskI-BX_VRu1E6n9YAA&usqp=CAU)
1. Calculate the distance from the Lidar (assumed to be at (0,0) coordinates) to each data point belonging to the reflector in a single laser scan.
2. Select the two (or more) closest data points to the Lidar.
3. Take the average of the two selected data points' coordinates to approximate the position where the laser beam is normal to the cylindrical reflector's surface.
4. Extend the line of sight from the incident normal beam by the known radius amount to locate the center coordinates of the reflector and thereby determine the distance from the Lidar.
This logic is coded in processBag.py under scripts folder 
### Collecting Data
The cylindrical reflector is positioned at intervals of 0.05m, ranging from 0.129m to 3m away from the Lidar. At each distance experiment, a Laser Scan message is recorded.
To collect the data, follow these steps:
Connect the Hokuyo UST-30LX Lidar to your laptop and bring up the hokuyo node along with rviz using the following command:
```
roslaunch calibration collectData.launch
```
Navigate to the test_bags folder and record the laser scan message after setting up the reflector at a specific distance. Record the data for 30 seconds and name the bag file according to the measured distance in millimeters. For example:
```
timeout 30 rosbag record -O exp_2620.bag /scan
```
### Binary Segmentation and executing method to estimate distance to lidar
To filter out data points not belonging to the reflector, we use the binsegBag.py script, which performs binary segmentation based on an intensity threshold.
To begin, navigate to the scripts folder and execute the binsegBag.py script as follows:
```
python3 binsegBag.py
```
Please ensure that the relative_path variable in the code is correctly set to the path containing all the scan message rosbags. For example:
```relative_path = 'Reflector-Calibration/intern_ws/src/calibration/test_bags' ```
The binary segmented rosbags will be created in the test_bags folder, labeled with the "bs_" prefix.
If you wish to delete the binary segmented rosbags, you can run the deleteBsBag.py script:
```
python3 deleteBsBag.py
```
Once the binary segmentation is complete, you can proceed to execute the processBag.py script, which applies the proposed method to estimate the distance to the Lidar. The script calculates the error against distance and plots the resulting graph.
To run the processBag.py script, use the following command:
```
python3 processBag.py
```
With these steps, you will perform binary segmentation to filter relevant data points, estimate the distance to the Lidar using the proposed method, and visualize the error against distance through a graph.

