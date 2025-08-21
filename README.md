# Autonomous-Ship

### Description
Up until this point, I've only experimented with web development and wasn't too high on it. At the same, I've been constantly reading about the Russo-Ukranian War, the Gaza War, and China's build up. This motivated me to get into defense tech and since I'm especially interested in autonomous systems, I wanted this project to be an opportunity to learn the fundamentals of a robot: ROS2, computer vision, sensor fusion, and CAD. This project will never be fully done because I will add more and more autonomous functionality to the boat. 

### Goal
Develop a ship that changes its speed and direction to follow a moving object.

### Results
### Accomplished Steps
1. Built boat in Autodesk Fusion.
2. Tried to detect objects by color. But, I quickly realized that this approach relied too much on lighting. Based on my goal, I then realized I needed to focus more on detecting object movement rather than object identificaiton. 
3. Used MOG2 background subtraction to detect object movement and linked it to servo motor. Servo motor recieves angular.z (yaw) from MOG2 to turn rudder to left/right to follow object.<br/>
4. Initially, the pi controlled both the ultrasonic sensor and the camera. However, the camera hogged the CPU causing sensor echos to be missed.
5. Moved the sensor to the Arduino. I still had to sync the sensor with the servo motor, but the Arduino made it much easier to control clock cycles.
6. Implemented ultrasonic sensor to change motor RPM based on distance from tracked object.


### Next Steps
1. Make the boat watertight to test it in water

