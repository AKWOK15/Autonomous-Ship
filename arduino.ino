// Arduino code: Servo control + Ultrasonic sensor with USB Serial communication
// Simplified version using millis() instead of timer interrupts
// No external libraries needed beyond Servo.h

#include <Servo.h>

// Pin definitions
const int SERVO_PIN = 9;        // Servo pin (matches your existing code)
const int TRIGGER_PIN = 8;      // Ultrasonic trigger
const int ECHO_PIN = 10;        // Ultrasonic echo

// Servo control variables
Servo myservo;
String receivedString = "";
float angleFloat;
boolean newData = false;
int currentAngle = 90;          // Track current position
int targetAngle = 90;           // Target position
unsigned long lastMoveTime = 0;
const int MOVE_DELAY = 10;      // Smooth movement delay

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
const unsigned long DISTANCE_SEND_INTERVAL = 200;  // Send distance every 200ms
const unsigned long ULTRASONIC_INTERVAL = 125;     // Trigger ultrasonic every 125ms

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Initialize servo
    myservo.attach(SERVO_PIN);
    myservo.write(90);
    
    // Initialize ultrasonic pins
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    
    Serial.println("Arduino Servo + Ultrasonic Ready (millis version)");
}

void loop() {
    // Handle servo control
    recvInfo();
    smoothMove();
    
    // Handle ultrasonic trigger timing
    handleUltrasonicTiming();
    
    // Handle ultrasonic processing
    processUltrasonicMeasurement();
    
    // Send distance data periodically
    sendDistanceData();
}

// Servo control functions
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
            }
        } else {
            // Build the string character by character
            receivedString += inChar;
        }
    }
}

void smoothMove() {
    if (newData && millis() - lastMoveTime > MOVE_DELAY) {
        servo_update_in_progress = true;  // Signal ultrasonic to wait
        
        if (currentAngle != targetAngle) {
            // Move one degree at a time for smooth motion
            if (currentAngle < targetAngle) {
                currentAngle++;
            } else if (currentAngle > targetAngle) {
                currentAngle--;
            }
            myservo.write(currentAngle);
            lastMoveTime = millis();
        } else {
            // Reached target
            newData = false;
        }
        
        servo_update_in_progress = false;  // Signal ultrasonic it's safe
    }
}

// Simple timing-based ultrasonic triggering (replaces timer interrupt)
void handleUltrasonicTiming() {
    unsigned long current_time = millis();
    
    if (current_time - last_ultrasonic_trigger >= ULTRASONIC_INTERVAL) {
        if (!ultrasonic_measuring && !servo_update_in_progress) {
            // Trigger ultrasonic measurement
            ultrasonic_measuring = true;
            digitalWrite(TRIGGER_PIN, HIGH);
            delayMicroseconds(10);
            digitalWrite(TRIGGER_PIN, LOW);
            start_echo_measurement();
            // Serial.print("handleUltrasonicTiming Finished");
        }
        last_ultrasonic_trigger = current_time;
    }
}

// Ultrasonic processing function
void processUltrasonicMeasurement() {
    // Serial.print("measurement_ready:");
    // Serial.println(measurement_ready);
    if (measurement_ready) {
        // Serial.print("!echo_timeout:");
        // Serial.println(!echo_timeout);
        // Serial.print("last_measurement_valid:");
        // Serial.println(last_measurement_valid);
        if (!echo_timeout && last_measurement_valid) {
            unsigned long echo_duration = echo_end_time - echo_start_time;
            unsigned int distance_cm = (echo_duration * 0.034) / 2;
            // Serial.print("distance_cm:");
            // Serial.println(distance_cm);
            // Validate measurement
            if (distance_cm > 0 && distance_cm < 400) {
                // Add to buffer for averaging
                distance_buffer[buffer_index] = distance_cm;
                buffer_index = (buffer_index + 1) % BUFFER_SIZE;
                if (buffer_index == 0) buffer_full = true;
            }
        }
        
        // Reset flags
        measurement_ready = false;
        echo_timeout = false;
        last_measurement_valid = false;
    }
}

// Send distance data to Pi periodically
void sendDistanceData() {
    unsigned long current_time = millis();
    
    if (current_time - last_distance_send >= DISTANCE_SEND_INTERVAL) {
        if (buffer_full || buffer_index > 0) {
            // Calculate average distance
            unsigned long sum = 0;
            int count = buffer_full ? BUFFER_SIZE : buffer_index;
            
            for (int i = 0; i < count; i++) {
                sum += distance_buffer[i];
            }
            
            unsigned int avg_distance = sum / count;
            
            // Send structured data to Pi
            // Serial.print("DIST:");
            // Serial.print(avg_distance);
            
            Serial.print("angleFloat:");
            Serial.println(angleFloat);
            Serial.print(",SERVO:");
            Serial.println(currentAngle);
        }
        
        last_distance_send = current_time;
    }
}

// Non-blocking distance measurement
void start_echo_measurement() {
    unsigned long timeout_start = micros();
    
    // Wait for echo start (with timeout check)
    while (digitalRead(ECHO_PIN) == LOW) {
        if (micros() - timeout_start > 30000) {  // 30ms timeout
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;            
            return;
        }
        
        // Yield if servo needs to update
        if (servo_update_in_progress) {
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;
            return;
        }
    }
    
    echo_start_time = micros();
    
    // Wait for echo end (with timeout and servo check)
    while (digitalRead(ECHO_PIN) == HIGH) {
        unsigned long current_time = micros();
        
        if (current_time - echo_start_time > 25000) {  // 25ms timeout
            echo_timeout = true;
            ultrasonic_measuring = false;
            measurement_ready = true;
            return;
        }
        
        // Check if servo update is needed - abort measurement if so
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
