#include "libHCSR04.hpp"
#include <pigpiod_if2.h>  // Changed from pigpio.h to pigpiod_if2.h
#include <iostream>
#include <thread>
#include <chrono>

HCSR04::HCSR04() : initialized_(false), pi_(-1) {}  // Added pi_ member

HCSR04::~HCSR04() {
    cleanup();
}

bool HCSR04::init(int trigger_pin, int echo_pin) {
    trigger_pin_ = trigger_pin;
    echo_pin_ = echo_pin;
    
    // Connect to pigpiod daemon instead of direct GPIO access
    pi_ = pigpio_start(NULL, NULL);  // Connect to local daemon
    if (pi_ < 0) {
        std::cerr << "Failed to connect to pigpiod daemon. Make sure pigpiod is running." << std::endl;
        std::cerr << "Run: sudo pigpiod" << std::endl;
        return false;
    }
    
    // Set pin modes using daemon interface
    set_mode(pi_, trigger_pin_, PI_OUTPUT);
    set_mode(pi_, echo_pin_, PI_INPUT);
    
    // Set trigger low initially
    gpio_write(pi_, trigger_pin_, 0);
    
    // Wait for sensor to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    initialized_ = true;
    return true;
}

double HCSR04::distance(int timeout_us) {
    if (!initialized_) {
        std::cerr << "HCSR04 not initialized!" << std::endl;
        return -1.0;
    }
    
    // Small delay between measurements
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Send trigger pulse
    gpio_write(pi_, trigger_pin_, 1);
    time_sleep(0.00001); // 10 microseconds (0.00001 seconds)
    gpio_write(pi_, trigger_pin_, 0);
    
    // Wait for echo to start (go HIGH)
    auto timeout_start = std::chrono::high_resolution_clock::now();
    while (gpio_read(pi_, echo_pin_) == 0) {
        auto elapsed = std::chrono::high_resolution_clock::now() - timeout_start;
        if (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() > timeout_us) {
            std::cerr << "Timeout waiting for echo start" << std::endl;
            return -1.0;
        }
    }
    
    // Record pulse length
    recordPulseLength();
    
    // Calculate distance
    auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time_ - start_time_).count();
    
    // Distance = (time × speed of sound) / 2
    // Speed of sound = 343 m/s = 0.034029 cm/µs
    double distance_cm = (pulse_duration * SOUND_SPEED_CM_PER_US) / 2.0;
    
    return distance_cm;
}

void HCSR04::recordPulseLength() {
    // Record start time when echo goes HIGH
    start_time_ = std::chrono::high_resolution_clock::now();
    
    // Wait for echo to go LOW and record end time
    while (gpio_read(pi_, echo_pin_) == 1) {
        // Busy wait - not ideal but needed for timing precision
    }
    end_time_ = std::chrono::high_resolution_clock::now();
}

void HCSR04::cleanup() {
    if (initialized_) {
        pigpio_stop(pi_);  // Disconnect from daemon instead of gpioTerminate()
        pi_ = -1;
        initialized_ = false;
    }
}
