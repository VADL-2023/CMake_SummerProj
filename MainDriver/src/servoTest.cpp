#include "sensors.h"
#include "config_IMU.hpp"
#include "pigpio.h"
#include <stddef.h>

uint16_t pulseMin = 500; // usecs
uint16_t pulseMax = 2500; // usecs
uint8_t servoRange = 180; // deg

// given an angle [yes mom, its in degrees] finds the specified pulse width to move there [for specified servo parameters)
float convertAngle2PulseWidth(float angle) {
    float x = (pulseMax - pulseMin)*angle/servoRange + (pulseMax + pulseMin)/2.0; // angle between +-90
    std::cout << "Pulse: " << x << std::endl;
    return x;
}

// Hi mom, this function serves to move a pair of servos in opposite CW/CCW directions (but same linear direction) by the same amount (this applies if the user is outside of the rocket facing radially inward for the AAC project) - Eric and Matt
void moveServoPair(uint8_t pin1, uint8_t pin2, float angle){
    float pulseWidth1 = convertAngle2PulseWidth(angle);
    float pulseWidth2 = convertAngle2PulseWidth(-angle);
    
    gpioServo(pin1, pulseWidth1);
    gpioServo(pin2, pulseWidth2);
}


int main(){
    std::cout << "Initialize GPIO Returned: " << gpioInitialise() << std::endl;
        
    uint8_t servoPin1 = 18;
    uint8_t servoPin2 = 19;
    uint8_t servoPin3 = 12;
    uint8_t servoPin4 = 13;
    
    // Initialize GPIO
    gpioSetMode(servoPin1, PI_OUTPUT);
    gpioSetMode(servoPin2, PI_OUTPUT);
    gpioSetMode(servoPin3, PI_OUTPUT);
    gpioSetMode(servoPin4, PI_OUTPUT);
    
    // move servos
    moveServoPair(servoPin1,servoPin2,12);
    gpioSleep(0,1,0);
    moveServoPair(servoPin1,servoPin2,-12);
    gpioSleep(0,1,0);

    // std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin1) << std::endl;
    
    gpioTerminate();

	return 0;
}
