#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <stddef.h>
#include <unistd.h>
#include <math.h>
#include <algorithm> 
#include <iostream>
#include "Log.h"
#include <chrono>
#include <thread>

// servo parameters
uint16_t pulseMin = 500; // [usecs] pulse width to send servo to one end of motion range
uint16_t pulseMax = 2500; // [usecs] pulse width to send servo to other end of motion range
uint8_t servoRange = 180; // [deg] possible range of motion of servo
uint8_t servoPinN = 18;
uint8_t servoPinE = 12;
uint8_t servoPinS = 13;
uint8_t servoPinW = 19;

float slow = 3;
float medium = 1;
float fast = 0.3;
uint8_t numSteps = 30;
float slowStep = slow/numSteps;
float mediumStep = medium/numSteps;
float fastStep = fast/numSteps;
float angleMax = 15;
float angleStep = angleMax/numSteps;


// given an angle [deg], finds the specified pulse width to move there (for specified servo parameters)
float convertAngle2PulseWidth(float angle) {
    return (pulseMax - pulseMin)*angle/servoRange + (pulseMax + pulseMin)/2.0; // angle between +-90
}

// moves a pair of servos in opposite CW/CCW directions (but same linear direction) by the same amount (you are outside of the rocket facing radially inward)
void moveServoPair(int8_t pin1, uint8_t pin2, float angle){
    float pulseWidth1 = convertAngle2PulseWidth(angle);
    float pulseWidth2 = convertAngle2PulseWidth(-angle);
    
    gpioServo(pin1, pulseWidth1);
    gpioServo(pin2, pulseWidth2);
}

// moves a single servo on [pin] to specified angle [deg]
void moveServo(uint8_t pin, float angle) {
    float pulseWidth = convertAngle2PulseWidth(angle);
    gpioServo(pin, pulseWidth);
}
    
int main(){
    
    VnSensor* mVN; 
    // IMU connection and configuration
    mVN = new VnSensor();
    try{
        mVN->connect(IMU_PORT, IMU_BAUD_RATE);
    } catch(std::exception){
        return 0;
    }

    // initialize servo pins as output
    gpioInitialise();
    gpioSetMode(servoPinN, PI_OUTPUT);
    gpioSetMode(servoPinE, PI_OUTPUT);
    gpioSetMode(servoPinS, PI_OUTPUT);
    gpioSetMode(servoPinW, PI_OUTPUT);
        
    // zero servos
    moveServo(servoPinN,0);
    moveServo(servoPinE,0);
    moveServo(servoPinS,0);
    moveServo(servoPinW,0);
        
    while(1) {
        for(int i=0; i < angleMax; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(mediumStep*1000*1000);
        }
        for(int i=angleMax; i > 0; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(mediumStep*1000*1000);
        }
        for(int i=0; i > -angleMax; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(mediumStep*1000*1000);
        }
        for(int i=-angleMax; i < 0; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(mediumStep*1000*1000);
        }
        for(int i=0; i < angleMax; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=angleMax; i > 0; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=0; i > -angleMax; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=-angleMax; i < 0; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=0; i < angleMax; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=angleMax; i > 0; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=0; i > -angleMax; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=-angleMax; i < 0; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(fastStep*1000*1000);
        }
        for(int i=0; i < angleMax; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(slowStep*1000*1000);
        }
        for(int i=angleMax; i > 0; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(slowStep*1000*1000);
        }
        for(int i=0; i > -angleMax; i--) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(slowStep*1000*1000);
        }
        for(int i=-angleMax; i < 0; i++) {
            moveServo(servoPinN,i);
            moveServo(servoPinE,i);
            moveServo(servoPinS,i);
            moveServo(servoPinW,i);
            usleep(slowStep*1000*1000);
        }
    }

            


    return 0;
}
