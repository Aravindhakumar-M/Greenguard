# Arm Controller Node

This Python script controls an arm mechanism based on depth sensor readings and moisture levels using the Raspberry Pi GPIO pins and the Robot Operating System (ROS).

## Prerequisites

Before running the code, make sure you have the following:

- Raspberry Pi with GPIO pins enabled.
- ROS installed on your system.
- Appropriate depth sensor and moisture level sensor connected to your Raspberry Pi.
- Required ROS packages set up if not already done.

## Usage

1. Run the ROS node by executing the following command:

```bash
rosrun <package_name> arm_controller.py
```

## The script will subscribe to two ROS topics:

`/depth:` Provides depth sensor readings.  
`/moisture_level:` Provides moisture level data.  

1. The script uses the depth sensor data to determine if the water level is below a certain threshold. Based on the depth reading, it controls the arm's movement as follows:  

2. If the depth is less than 5 (indicating low water level):
   - If the depth sensor detects water and the arm control signal is high, the arm movement is stopped.
   - If the depth sensor detects water and the flag is set to False, the arm moves in one direction (e.g., to dispense water).
   - If the depth sensor detects water and the flag is set to True, the arm moves in the opposite direction (e.g., to retract).  
3. The script also monitors the moisture level data. If the moisture level is greater than 20, it briefly retracts the arm mechanism for a specified time (2 seconds). This helps avoid overwatering plants.
