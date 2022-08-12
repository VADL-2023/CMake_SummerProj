#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <stddef.h>
#include <unistd.h> // has sleep() function
#include <math.h>
#include <algorithm> // array functions?
#include <iostream>
#include <Log>

// conversion factors
float ft2m = 0.3048; // [m/ft]
float m2ft = 1/ft2m; // [ft/m]
float C2K = 273.15; // Celsius to Kelvin
float km2m = 0.001; // [km/m]

// constants
float R = 287; // [kg/J/K] universal gas constant
float B = 6.5*km2m; //[K/m] variation of temperature within the troposphere

// flight parameters
float h0 = 522*ft2m;
uint8_t finTiltAngle = 12; // [deg] fixed tilt angle for airfoil activation 
uint8_t tBurn = 1.6; //[s] motor burn time
float accelRoof = 3; // how many g's does the program need to see in order for launch to be detected
float samplingFrequency = 20; // [Hz] how fast does the IMU sample data
float burnSafetyMargin = 3; // what fraction of t_burn will we check acceleration samples for
int numDataPointsChecked4Launch = ceil(tBurn/burnSafetyMargin*samplingFrequency); // how many acceleration points are averaged to see if data set is over accelRoof
int numDataPointsChecked4Apogee = 10; // how many altitude points must a new max not be found for apogee to be declared
float zDeploy = 650*ft2m; // [ft] altitude at which fins will deploy above ground level

// servo parameters
uint16_t pulseMin = 500; // [usecs] pulse width to send servo to one end of motion range
uint16_t pulseMax = 2500; // [usecs] pulse width to send servo to other end of motion range
uint8_t servoRange = 180; // [deg] possible range of motion of servo

// calibration parameters
uint16_t numSampleReadings = 25; // amount of samples taken and averaged to find ground P and T
float servoTestTiltWaitTime = 1; // [s] amount of time between servo movement tests
float servoTestBeginWaitTime = 1; // [s] amount of time before servo tests begin

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
    gpioServo(pin,pulseWidth);
}

// given pin and angle, activates test protocol for a single servo
void testServo(uint8_t pin, float angle) {
    moveServo(pin, angle);
    gpioSleep(0,servoTestTiltWaitTime,0);
    moveServo(pin, -angle);
    gpioSleep(0,servoTestTiltWaitTime,0);
    moveServo(pin, 0); // end by moving servo to middle
    gpioSleep(0,servoTestTiltWaitTime,0);
}

// given T0 [K], P0 [kPa], g0 [m/s^2], P [kPa], returns altitude above ground level
// 0 indicates baseline measurement, R and B are constants
float pressure2Altitude(float T0, float P0, float g0, float P) {
    return T0/B*(pow(P/P0,-R*B/g0) - 1);
}

// given a float array, calculates the average of all the arrays values
float calcArrayAverage(float array[], int size) {
    float sum;
    for (int i = 0; i <= size; ++i) {
        sum += array[i];
    }
    return sum/size;
}
    
int main(){
    /* P R E - F L I G H T  S T A G E*//////////////////////////////////
    // declare variables which may get overwritten several times during go-nogo
    float pressureSum;
    float tempSum;
    float gravSum;
    float P0; // pressure at ground level
    float T0; // temperature at ground level
    float g0; // gravity at ground level
    
    // begin GO-NOGO Protocol
    string go = "NOGO";
    while (go != "GO") {
        
        // check GPIO pin starts
        if(gpioInitialise() <= 0) {
            std::cout << "GPIO Failed to Initialize" << std::endl;
        } else {
            std::cout << "GPIO Initialized" << std::endl;
        }

        // initialize servo pins as output
        gpioSetMode(servoPinN, PI_OUTPUT);
        gpioSetMode(servoPinE, PI_OUTPUT);
        gpioSetMode(servoPinS, PI_OUTPUT);
        gpioSetMode(servoPinW, PI_OUTPUT);
    
        // IMU Connection and Configuration
        VnSensor* mVN = new VnSensor();
        
        // initialize log (erases every loop?)
        Log2 Log("Flight Data Log", "Program Data Log ", mVN);
    
        std::cout << "IMU Connecting" << std::endl;
        mVN->connect(IMU_PORT,IMU_BAUD_RATE);
        if (!mVN->isConnected()){
            throw "IMU Failed to Connect";
        }else{
            std::cout << "IMU Connected" << std::endl;
        }
        
        // test all 4 servos
        std::cout << "Testing Servo Activation" << std::endl;
        sleep(servoTestBeginWaitTime);
        std::cout << "Testing North..." << std::endl;
        testServo(servoPinN,finTiltAngle);
        std::cout << "Testing East..." << std::endl;
        testServo(servoPinE,finTiltAngle);
        std::cout << "Testing South..." << std::endl;
        testServo(servoPinS,finTiltAngle);
        std::cout << "Testing West..." << std::endl;
        testServo(servoPinW,finTiltAngle);
        
        ImuMeasurementsRegister response;
        
        // calibrate ground level pressure and temperature
        pressureSum = 0;
        tempSum = 0;
        gravSum = 0;
        std::cout << "Calibrating Baseline Parameters. Hold Still." << std::endl;
        for (int i = 0; i < numSampleReadings; ++i){
            response = mVN->readImuMeasurements();
            pressureSum += response.pressure;
            tempSum += response.temp;
            gravSum += sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2));
        }
        P0 = pressureSum/numSampleReadings;
        T0 = tempSum/numSampleReadings + C2K;
        g0 = gravSum/numSampleReadings;
        std::cout << "Calibrated Temperature: " << T0 - C2K << " C" << std::endl;
        std::cout << "Calibrated Pressure: " << P0 << " kPa" << std::endl;
        std::cout << "Calibrated Gravity: " << g0 << " m/s^2" << std::endl;
        std::cout << "Are we a GO for flight?" << std::endl;
        std::cin >> go;
        
        // if no-go, undo initializations so we can try again
        if(go != "GO") {
            if (IMU_ACTIVE){
                mVN->disconnect();
                std::cout << "IMU Disconnected" << std::endl;
            }
            delete mVN;
            gpioTerminate();
            std::cout << "GPIO Terminated" << std::endl;
        } else { // if go, disconnect pointer so normal object can be created // this is weird pls take a look at this // I was getting object errors because this mVN is declared within the while loop, so it is not recognized outside of it. Destroy and create a new one. But does that defeat the whole checking initialization process?
            if (IMU_ACTIVE){
                mVN->disconnect();
                std::cout << "IMU Disconnected" << std::endl;
                // delete mVN; // deleting older pointer here gives an error for some reason
            }
        }
    }
    std::cout << "Pre-Flight Stage Completed" << std::endl;
    
    VnSensor mVN;
    mVN.connect(IMU_PORT,IMU_BAUD_RATE);
    ImuMeasurementsRegister response;
    if (!mVN.isConnected()){
        throw "IMU Failed to Connect";
        std::cout << "DO NOT CONTINUE FLIGHT" << std::endl;
    }else{
        std::cout << "IMU Connected" << std::endl;
    }
    
    /* L A U N C H  S T A G E */////////////////////////////////////////
    std::cout << "Ready for Assembly and Launch Rail" << std::endl;
    
    float accelArray [numDataPointsChecked4Launch] = {};
    float accelAvg;
    
    while(accelAvg < accelRoof*g0){
        response = mVN.readImuMeasurements(); // record data
        accelArray[0] = sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2)); // total accel vector
        accelAvg = calcArrayAverage(accelArray, numDataPointsChecked4Launch);
        
        for (int i = 0; i < numDataPointsChecked4Launch; ++i) { // shift array values down
            accelArray[i+1] = accelArray[i];
        }
    }
    std::cout << "Average acceleration exceeded " << accelRoof*g0 << " m/s^2 over " << numDataPointsChecked4Launch << " data points" << std::endl;
    std::cout << "Rocket Has Launched" << std::endl;
        
    std::cout << "Waiting for Motor Burn Time" << std::endl;
    sleep(tBurn);
    
    float zCurrent;
    
    std::cout << "Actively Checking Altitude" << std::endl;
    while (zCurrent < zDeploy) {
        response = mVN.readImuMeasurements();
        zCurrent = pressure2Altitude(T0,P0,g0,response.pressure);
    }
    std::cout << "Deployment Altitude Reached" << std::endl;
    
    moveServoPair(servoPinN,servoPinS,finTiltAngle);
    std::cout << "Fins Deployed" << std::endl;
    
    /* C O A S T I N G  S T A G E*//////////////////////////////////////
    
    float zCurrentArray [numDataPointsChecked4Apogee] = {};
    float maxAltitude;
    int samplesSinceMaxHasChanged;
    
    // loop runs until we havent hit a new max altitude for numDataPointsChecked
    // i.e. we are not going up anymore
    while(samplesSinceMaxHasChanged < numDataPointsChecked4Apogee) {
        response = mVN.readImuMeasurements();
        zCurrent = pressure2Altitude(T0,P0,g0,response.pressure);
        zCurrentArray[0] = zCurrent;
        
        if (zCurrent >= maxAltitude) {
            maxAltitude = zCurrent;
            samplesSinceMaxHasChanged = 0;
        } else {
            ++samplesSinceMaxHasChanged;
        }
        
        // shift array values
        for (int i = 0; i < numDataPointsChecked4Apogee; ++i) {
            zCurrentArray[i+1] = zCurrentArray[i];
        }
    }
    
    std::cout << "Altitude has not reached a new max for " << numDataPointsChecked4Apogee << " samples... retracting fins now." << std::endl;
    
    moveServoPair(servoPinN,servoPinS,0);
    std::cout << "Fins Undeployed" << std::endl;
    
    /* D E S C E N T  S T A G E *///////////////////////////////////////
    // all that needs to happen here is data keeps being saved and foils are kept at 0 angle
    
    if (IMU_ACTIVE){
        mVN.disconnect();
        std::cout << "IMU: Disconnected" << std::endl;
	}
    // delete mVN; // delete only needed for pointers
    gpioTerminate();
    
    return 0;
}
