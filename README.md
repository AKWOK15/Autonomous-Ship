# Autonomous-Ship
In Progress
### Project Goal
Develop an autonomous ship that changes its speed and direction to follow moving object.
  - CAD
  - Embedded Systems/C++
  - Sensor Fusion - Kalman filter
  - Computer Vision
  - ROS2

### Accomplished Steps
1. Built boat 1.0 in Autodesk
<img width="754" alt="Screenshot 2025-05-31 at 1 35 24 PM" src="https://github.com/user-attachments/assets/e0b49fca-9878-48d5-90a6-fdf6400a827d" />
<br/><br/>
2. Used MOG2 background subtraction to detect object movement. Linked it to servo motor, so that servo motor will adjust rudder to follow object.<br/>
3. Implemented ultrasonic sensor to change motor RPM based on distance from tracked object. 


### Next Steps
1. Boat 1.0 floated by itself but sank once loaded with the weight of the electronics.\
  a. Since this 600 gram boat will be powered by a 3-6V motor, it will only move around 1-3 mph. So, I need to decrease hull width to maximize hydrodynamics and therefore speed.\
  b. SimScale couldn't simulate a boat sinking in water. Need to find a new simulation platform to test boat sinkage, so I know where I should place propeller and how to best distribute electronics within boat to ensure balance.\
  c. Add rudder and propeller.\
  d. Find optimal spot on hull to attach pi camera and ultrasonic sensor.

