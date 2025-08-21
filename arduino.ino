// Debug version to see exactly what Arduino receives

#include <Servo.h>

const int SERVO_PIN = 8;
const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;
// PWM Pin
const int MOTOR_PIN = 3;

// Servo control variables
Servo myservo;
String receivedString = "";
float angleFloat;
boolean newData = false;
int currentAngle = 90;
int targetAngle = 90;
unsigned long lastMoveTime = 0;
const int MOVE_DELAY = 10;

// Ultrasonic measurement variables
unsigned long echo_start_time = 0;
unsigned long echo_end_time = 0;
bool measurement_ready = false;
bool echo_timeout = false;
bool ultrasonic_measuring = false;

// Measurement buffer for averaging
const int BUFFER_SIZE = 5;
unsigned int distance_buffer[BUFFER_SIZE];
int buffer_index = 0;
bool buffer_full = false;

// Timing coordination
bool servo_update_in_progress = false;
bool last_measurement_valid = false;
unsigned long last_distance_send = 0;
unsigned long last_ultrasonic_trigger = 0;
const unsigned long DISTANCE_SEND_INTERVAL = 200;
const unsigned long ULTRASONIC_INTERVAL = 125;

void setup() {
    Serial.begin(9600);
    
    myservo.attach(SERVO_PIN);
    myservo.write(90);
    
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    
    Serial.println("Arduino Servo + Ultrasonic Ready (DEBUG VERSION)");
}

void loop() {
    recvInfo();
    smoothMove();
    handleUltrasonicTiming();
    processUltrasonicMeasurement();
    motorSpeed();
}

// DEBUG VERSION: Show exactly what we receive
void recvInfo() {
    while (Serial.available() > 0) {
        char inChar = Serial.read();
        
        if (inChar == '\n') {
            // End of message - process the complete string
            if (receivedString.length() > 0) {
                angleFloat = receivedString.toFloat();
                
                targetAngle = constrain((int)angleFloat, 0, 180);
                
                newData = true;
                receivedString = "";  // Clear for next message
                
                Serial.print("Target Angle:");
                Serial.println(targetAngle);
            } 
        } else {
            // Build the string character by character
            receivedString += inChar;
        }
    }
}

void smoothMove() {
    if (newData && millis() - lastMoveTime > MOVE_DELAY) {
        servo_update_in_progress = true;
        
        if (currentAngle != targetAngle) {
            if (currentAngle < targetAngle) {
                currentAngle++;
            } else if (currentAngle > targetAngle) {
                currentAngle--;
            }
            myservo.write(currentAngle);
            lastMoveTime = millis();
        } else {
            newData = false;
        }
        
        servo_update_in_progress = false;
    }
}

void handleUltrasonicTiming() {
    unsigned long current_time = millis();
    
    if (current_time - last_ultrasonic_trigger >= ULTRASONIC_INTERVAL) {
        if (!ultrasonic_measuring && !servo_update_in_progress) {
            ultrasonic_measuring = true;
            digitalWrite(TRIGGER_PIN, HIGH);
            delayMicroseconds(10);
            digitalWrite(TRIGGER_PIN, LOW);
            start_echo_measurement();
        }
        last_ultrasonic_trigger = current_time;
    }
}

void processUltrasonicMeasurement() {
    if (measurement_ready) {
        if (!echo_timeout && last_measurement_valid) {
            unsigned long echo_duration = echo_end_time - echo_start_time;
            unsigned int distance_cm = (echo_duration * 0.034) / 2;
            
            if (distance_cm > 0 && distance_cm < 400) {
                distance_buffer[buffer_index] = distance_cm;
                buffer_index = (buffer_index + 1) % BUFFER_SIZE;
                if (buffer_index == 0) buffer_full = true;
            }
        }
        
        measurement_ready = false;
        echo_timeout = false;
        last_measurement_valid = false;
    }
}

void motorSpeed(){
  int avgDistance = 0;
  
  // Calculate average distance
  for (int x = 0; x < BUFFER_SIZE; x++){
    avgDistance += distance_buffer[x];
  }
  avgDistance = avgDistance / BUFFER_SIZE;
  
  Serial.print("Distance: ");
  Serial.print(avgDistance);
  Serial.print(" cm, ");
  
  // Map distance to PWM value (0-255)
  // Closer objects = slower motor, farther objects = faster motor
  int motorSpeed;
  
  if (avgDistance <= 10) {
    // Very close - motor off
    motorSpeed = 0;
  }
  else if (avgDistance >= 100) {
    // Far away - maximum speed
    motorSpeed = 255;
  }
  else {
    // Map distance 10-100cm to PWM 0-255
    // Linear mapping: closer = slower, farther = faster
    motorSpeed = map(avgDistance, 10, 100, 0, 255);
  }
  
  // Apply PWM to motor
  analogWrite(MOTOR_PIN, motorSpeed);
  
  Serial.print("Motor PWM: ");
  Serial.println(motorSpeed);
}

void start_echo_measurement() {
    unsigned long timeout_start = micros();
    
    while (digitalRead(ECHO_PIN) == LOW) {
        if (micros() - timeout_start > 30000) {
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;            
            return;
        }
        
        if (servo_update_in_progress) {
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;
            return;
        }
    }
    
    echo_start_time = micros();
    
    while (digitalRead(ECHO_PIN) == HIGH) {
        unsigned long current_time = micros();
        
        if (current_time - echo_start_time > 25000) {
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;
            return;
        }
        
        if (servo_update_in_progress) {
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;
            return;
        }
    }
    
    echo_end_time = micros();
    last_measurement_valid = true;
    ultrasonic_measuring = false;
    measurement_ready = true;
}
