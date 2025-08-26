# Autonomous-Ship

### Description
Up until this point, I've only experimented with web development and wasn't too high on it. At the same, I've been constantly reading about the Russo-Ukranian War, the Gaza War, and China's build up. This motivated me to get into defense tech and since I'm especially interested in autonomous systems, I wanted this project to be an opportunity to learn the fundamentals of a robot: ROS2, computer vision, sensor fusion, and CAD. This project will never be fully done because I will add more and more autonomous functionality to the boat. 

### Goal
Develop a ship that changes its speed and direction to follow a moving object.

### Results
![IMG_5083](https://github.com/user-attachments/assets/0d5a5739-c91d-4640-bfbd-73bdded864d7)
![IMG_5082](https://github.com/user-attachments/assets/29295b17-519f-49a8-bfd0-91d051764ad6)
![IMG_5081](https://github.com/user-attachments/assets/838e760f-7b37-4663-82df-fd28c33415fa)


https://github.com/user-attachments/assets/ad108579-19cb-4ddf-bd21-bdc428876bc9

 Chose MOG2 over KNN because it detects objects 3x faster from start up.


### Accomplished Steps
1. Built boat in Autodesk Fusion.
2. Tried to detect objects by color. But, I quickly realized that this approach relied too much on lighting. Based on my goal, I then realized I needed to focus more on detecting object movement rather than object identificaiton. 
3. Used MOG2 background subtraction to detect object movement and linked it to servo motor. Servo motor recieves angular.z (yaw) from MOG2 to turn rudder to left/right to follow object.<br/>
4. Initially, the pi controlled both the ultrasonic sensor and the camera. However, the camera hogged the CPU causing sensor echos to be missed.
5. Moved the sensor to the Arduino. I still had to sync the sensor with the servo motor, but the Arduino made it much easier to control clock cycles.
6. Implemented ultrasonic sensor to change motor RPM based on distance from tracked object.

### Lessons Learned
1. Hardware + Software > Software. I'd intern from 9-5, then work on this from 5-8. If this was a website project, I would not have been able to stay motivated, but this project was/is so much more fun. 
2. Perception is really hard because there are so many external factors that can affect it: lighting being one.
3. I now understand why hardware development takes so much longer than software. 3D printing the housing (boat hull) for the hardware took hours, whenever there's a bug, it could be a bug in the software or on the hardware, and I frequently had to wait for ordered parts before I could continue.

### Next Steps
1. Make the boat watertight to test it in water

