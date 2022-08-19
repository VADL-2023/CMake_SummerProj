#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <stddef.h>
#include <unistd.h> // has sleep() function
#include <math.h>
#include <algorithm> // array functions?
#include <iostream>
#include "Log.h"

// conversion factors
float ft2m = 0.3048; // [m/ft]
float m2ft = 1/ft2m; // [ft/m]
float C2K = 273.15; // Celsius to Kelvin
float km2m = 0.001; // [km/m]

// constants
float R = 287; // [kg/JK] universal gas constant
float B = 6.5*km2m; //[K/m] variation of temperature within the troposphere

// fixed flight parameters
uint8_t airfoilTiltAngle = 12; // [deg] fixed tilt angle for airfoil activation 
float tBurn = 1.6; //[s] motor burn time
float samplingFrequency = 20; // [Hz] how fast does the IMU sample data

// possibly variable flight parameters (stuff we might change)
float accelRoof = 1.2; // how many g's does the program need to see in order for launch to be detected
int numDataPointsChecked4Launch = 10; // how many acceleration points are averaged to see if data set is over accelRoof
int numDataPointsChecked4Apogee = 10; // how many altitude points must a new max not be found for apogee to be declared
int numDataPointsChecked4Landing = 10*samplingFrequency; // how many altitude points must a new min not be found for landing to be declared
float zDeploy = 650*ft2m; // [m] altitude at which fins will deploy above ground level
bool servoTest = true; // whether or not to test actuation range of servos during GO/NOGO
bool restart = false; // tells the program whether or not we NO-GOed

// servo parameters
uint16_t pulseMin = 500; // [usecs] pulse width to send servo to one end of motion range
uint16_t pulseMax = 2500; // [usecs] pulse width to send servo to other end of motion range
uint8_t servoRange = 180; // [deg] possible range of motion of servo

// calibration parameters
uint16_t numSampleReadings = 60; // amount of samples taken and averaged to find ground P and T
float servoTestTiltWaitTime = 1; // [s] amount of time between servo movement tests
float servoTestBeginWaitTime = 1; // [s] amount of time before servo tests begin
int imuWait = 60; //number of samples to get from IMU before actually starting to use + save data

// define 4 servo pins
uint8_t servoPinN = 18;
uint8_t servoPinE = 12;
uint8_t servoPinS = 13;
uint8_t servoPinW = 19;

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

// given pin and angle, activates test protocol for a single servo
void testServo(uint8_t pin, float angle) {
    moveServo(pin, angle);
    gpioSleep(0,servoTestTiltWaitTime,0);
    moveServo(pin, -angle);
    gpioSleep(0,servoTestTiltWaitTime,0);
    moveServo(pin, 0); // end by moving servo to middle
    gpioSleep(0, servoTestTiltWaitTime,0);
}

// given T0 [K], P0 [kPa], g0 [m/s^2], P [kPa], returns altitude above ground level
// 0 indicates baseline measurement, R and B are constants
float pressure2Altitude(float T0, float P0, float g0, float P) {
    return T0/B*(pow(P/P0,-R*B/g0) - 1);
}

// given a float array, calculates the average of all the arrays values
float calcArrayAverage(float array[], int size) {
    float sum;
    for (int i = 0; i < size; ++i) {
        sum += array[i];
    }
    return sum/size;
}

// returns current time in milliseconds since epoch
double getCurrentTime(){
    return double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

// functional the same as sleep() command but still records and logs IMU data
void activeSleep(float sleepTime, VnSensor* imu, ImuMeasurementsRegister &response, Log &log){
    double currentTime = getCurrentTime();
    double endTime = sleepTime*1000 + currentTime;
    while (currentTime < endTime){
        response = imu->readImuMeasurements();
        log.write(response);
        currentTime = getCurrentTime();
    }
}    

    
int main(){
            // check GPIO pin starts
        if(gpioInitialise() <= 0) {
            std::cout << "gpio is not working" <<std::endl;
        } else {
            std::cout << "gpio is working" <<std::endl;
        }
        
        
        while(true){
            moveServo(servoPinN, 0);
            //sleep(1);
            moveServo(servoPinE, 0);
            //sleep(1);
            moveServo(servoPinS, 0);
            //sleep(1);
            moveServo(servoPinW, 0);
            sleep(1);
        }
        
        
        std::cout << "servos have been zeroed" <<std::endl;
}
