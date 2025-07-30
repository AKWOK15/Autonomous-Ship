#include <Servo.h>

Servo myservo;
String receivedString = "";
boolean newData = false;
int currentAngle = 90;  // Track current position
int targetAngle = 90;   // Target position
unsigned long lastMoveTime = 0;
const int MOVE_DELAY = 15;  // Smooth movement delay

void setup() {
    Serial.begin(9600);
    myservo.attach(9);
    myservo.write(90);  // Start at center position
    Serial.println("Arduino Servo Ready");
}

void loop() {
    recvInfo();
    smoothMove();
}

void recvInfo() {
    while (Serial.available() > 0) {
        char inChar = Serial.read();
        
        if (inChar == '\n') {
            // End of message - process the complete string
            if (receivedString.length() > 0) {
                float angleFloat = receivedString.toFloat();
                targetAngle = constrain((int)angleFloat, 0, 180);
                
                Serial.print("Received angle: ");
                Serial.println(targetAngle);
                
                newData = true;
                receivedString = "";  // Clear for next message
            }
        } else {
            // Build the string character by character
            receivedString += inChar;
        }
    }
}

void smoothMove() {
    if (newData && millis() - lastMoveTime > MOVE_DELAY) {
        if (currentAngle != targetAngle) {
            // Move one degree at a time for smooth motion
            if (currentAngle < targetAngle) {
                currentAngle++;
            } else if (currentAngle > targetAngle) {
                currentAngle--;
            }
            
            myservo.write(currentAngle);
            lastMoveTime = millis();
            
            Serial.print("Moving to: ");
            Serial.println(currentAngle);
        } else {
            // Reached target
            newData = false;
            Serial.println("Target reached");
        }
    }
}

// #include <Servo.h>

// Servo myservo;  // create servo object to control a servo
// // twelve servo objects can be created on most boards

// int pos = 0;    // variable to store the servo position

// void setup() {
//   myservo.attach(9);  // attaches the servo on pin 9 to the servo object
// }

// void loop() {
//   for (pos = 0; pos <= 180; pos += 20) { // goes from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//     delay(20);                       // waits 15ms for the servo to reach the position
//   }
//   for (pos = 180; pos >= 0; pos -= 20) { // goes from 180 degrees to 0 degrees
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//     delay(20);                       // waits 15ms for the servo to reach the position
//   }
// }
