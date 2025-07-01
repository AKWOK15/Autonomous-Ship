#ifndef LIBHCSR04_HPP
#define LIBHCSR04_HPP

#include <chrono>
#include <memory>

class HCSR04 {
public:
    HCSR04();
    ~HCSR04();
    
    bool init(int trigger_pin, int echo_pin);
    double distance(int timeout_us = 1000000);
    void cleanup();

private:
    void recordPulseLength();
    
    int trigger_pin_;
    int echo_pin_;
    bool initialized_;
    int pi_;   
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point end_time_;
    
    static constexpr double SOUND_SPEED_CM_PER_US = 0.034029; // cm/µs at 20°C
};

#endif // LIBHCSR04_HPP
