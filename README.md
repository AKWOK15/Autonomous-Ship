# Autonomous-Ship

### Description
This past spring, I attended Special Operations Forces (SOF) Week, where I watched an autonomous ship zip around a lake. Immediately, I thought, that's what I want to do. At the time, I had no hardware experience. Consequently, this project was a way for me to dive off the deep end and learn hardware and the fundamentals of a robot: computer vision, controls, sensor routing, and sensor fusion.

### Goal - Intercept algorithm
The boat finds the biggest moving object and tries to intercept it. The greater the distance between it and the tracked object, the faster the boat will travel. If the object goes left, then the boat goes left. Same for the right. 


### Demo
https://github.com/user-attachments/assets/e6fce736-122a-4a18-9390-2b3bc0ab2645

Scene 1: Proving the boat moves\
2. Propeller RPM changes based on distance to object (me)\
3. Rudder tracks object\
4. The boat should have intercepted me. Instead, the pool's ripples threw off its direction. Some combination of my hull shape, 3-6 volt DC motor, and propeller is at fault for being so susceptible to tiny ripples. (I'm hypothesizing that this is mostly a motor problem). 

### System Design
<img width="930" height="1079" alt="Boat Diagram" src="https://github.com/user-attachments/assets/1779fcef-040b-46ca-a9b7-2517a5d5f5d7" />

### Next Steps
1. Source a more powerful motor and Raspberry Pi battery pack.\
   a. Ensure my motor is powerful enough to overcome pool ripples. 
3. Waterproof the boat. I initially used a silicon sealant, but it created too much friction with my motor shaft. So, I then used grease, but water seeped in as soon as the motor shaft rotated. 
4. Refine the computer vision to ensure the rudder can more accurately track the biggest moving object.
5. Right now, propeller speed depends solely on the ultrasonic sensor. So if I make a sharp turn, the ultrasonic sensor won't detect me, and the propeller will stop. Trajectory estimation might be able to fix this?
