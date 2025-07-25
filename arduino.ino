#include <Servo.h>

Servo myservo;
char receivedChar;
int pos = 0;
boolean newData = false;

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
}

void loop() {
  recvInfo();
  changeAngle();
}

void recvInfo(){
  if (Serial.available() > 0){
    receivedChar = Serial.read();
    newData = true;
  }
}

void changeAngle(){
  //receivedChar as ASCII so subtracting it by char 0 turns it into an int
  int angle = (receivedChar - '0');
  while(newData == true){
    myservo.write(angle);
    delay(500);
    newData = false;
  }
}