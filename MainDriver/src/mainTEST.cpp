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

// flight parameters
float h0 = 522*ft2m; // [m] launch site altitude ASL
uint8_t airfoilTiltAngle = 12; // [deg] fixed tilt angle for airfoil activation 
float tBurn = 1.6; //[s] motor burn time
float accelRoof = 1.6; // how many g's does the program need to see in order for launch to be detected
float samplingFrequency = 20; // [Hz] how fast does the IMU sample data
float burnSafetyMargin = 3; // what fraction of t_burn will we check acceleration samples for
int numDataPointsChecked4Launch = ceil(tBurn/burnSafetyMargin*samplingFrequency); // how many acceleration points are averaged to see if data set is over accelRoof
int numDataPointsChecked4Apogee = 5; // how many altitude points must a new max not be found for apogee to be declared
int numDataPointsChecked4Landing = 10*samplingFrequency; // how many altitude points must a new min not be found for landing to be declared
float zDeploy = 550*ft2m; // [m] altitude at which fins will deploy above ground level

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
    for (int i = 0; i <= size; ++i) {
        sum += array[i];
    }
    return sum/size;
}

// returns current time in milliseconds since epoch
double getCurrentTime(){
    return double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

// functional the same as sleep() command while still recording and logging IMU data
// not working due to "use of deleted function" idk andrew help me
// has something to do with trying to implicity define the static variable startTime probably
/*
void activeSleep(int sleepTime, VnSensor* imu, ImuMeasurementsRegister response, Log log){
    double currentTime = getCurrentTime();
    double endTime = sleepTime + getCurrentTime();
    while (currentTime < endTime){
        response = imu->readImuMeasurements();
        log.write(response);
        currentTime = getCurrentTime();
    }
}    
  */  

int main(){
    /* P R E - F L I G H T  S T A G E*//////////////////////////////////
    
    // declare variables which may get overwritten several times during go-nogo
    float pressureSum;
    float tempSum;
    float gravSum;
    float P0; // pressure at ground level
    float T0; // temperature at ground level
    float g0; // gravity at ground level
    double startTime; //milliseconds since epoch
    
    VnSensor* mVN; 
    ImuMeasurementsRegister response;
    
    startTime = getCurrentTime();
    Log mLog("Flight Data Log MAINTEST 6", "Program Data Log MAINTEST 6", mVN, startTime);
    
    mLog.write("Date: 8/13");
    mLog.write("Flight Name: MainTEST 6\n");
    mLog.write("Verify Critical Parameters: ");
    mLog.write("Deployment Altitude: " + to_string(zDeploy*m2ft) + " Feet AGL");
    mLog.write("Deployment Altitude: " + to_string(zDeploy) + " Meters AGL");
    mLog.write("Deployment Angle: " + to_string(airfoilTiltAngle) + " Degrees");
    mLog.write("Motor Burn Time: " + to_string(tBurn) + " Seconds");
    mLog.write("Motor Burn Safety Factor: " + to_string(burnSafetyMargin));
    mLog.write("Trigger Acceleration: " + to_string(accelRoof) + " g");
    mLog.write("Launch Detection Samples: " + to_string(numDataPointsChecked4Launch));
    mLog.write("Apogee Detection Samples: " + to_string(numDataPointsChecked4Apogee));
    mLog.write("Landing Detection Samples: " + to_string(numDataPointsChecked4Landing));
    mLog.write("-----------------------------------\n\n\n");
    sleep(1);

    // begin GO-NOGO Protocol
    string go = "NOGO";
    
    while (go != "GO") {
        
        // IMU Connection and Configuration
        mVN = new VnSensor();
        
        // check GPIO pin starts
        if(gpioInitialise() <= 0) {
            mLog.write("GPIO Failed to Initialize");
        } else {
            mLog.write("GPIO Initialized");
        }

        // initialize servo pins as output
        gpioSetMode(servoPinN, PI_OUTPUT);
        gpioSetMode(servoPinE, PI_OUTPUT);
        gpioSetMode(servoPinS, PI_OUTPUT);
        gpioSetMode(servoPinW, PI_OUTPUT);
        
        mLog.write("IMU Connecting");
        mVN->connect(IMU_PORT,IMU_BAUD_RATE);
        if (!mVN->isConnected()){
            throw "IMU Failed to Connect";
        }else{
            mLog.write("IMU Connected");
        }
        
        // test all 4 servos
        mLog.write("Testing Servo Activation");
        sleep(servoTestBeginWaitTime);
        mLog.write("Testing North...");
        testServo(servoPinN,airfoilTiltAngle);
        mLog.write("Testing East...");
        testServo(servoPinE,airfoilTiltAngle);
        mLog.write("Testing South...");
        testServo(servoPinS,airfoilTiltAngle);
        mLog.write("Testing West...");
        testServo(servoPinW,airfoilTiltAngle);
        
        //timeTest(startTime, 2);
        
        // calibrate ground level pressure and temperature
        pressureSum = 0;
        tempSum = 0;
        gravSum = 0;
        mLog.writeTime("Calibrating Baseline Parameters. Hold Still.");
        sleep(1);
        
        for (int i = 0; i < numSampleReadings; ++i){
            response = mVN->readImuMeasurements();
            mLog.write(response);
            pressureSum += response.pressure;
            tempSum += response.temp;
            gravSum += sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2));
        }
        
        P0 = pressureSum/numSampleReadings;
        T0 = tempSum/numSampleReadings + C2K;
        g0 = gravSum/numSampleReadings;
        
        mLog.write("Calibrated Temperature: " + to_string(T0 - C2K) + " C");
        mLog.write("Calibrated Pressure: " + to_string(P0) + " kPa");
        mLog.write("Calibrated Gravity: " + to_string(g0) + " m/s^2");
        mLog.saveBaselineParameters(R, B, P0, T0, g0);
        mLog.writeTime("Are we a GO for flight?");
        std::cin >> go;
        mLog.writeTime(go);
        
    }
    
    mLog.writeTime("Pre-Flight Stage Completed");
    
    
    /* L A U N C H  S T A G E */////////////////////////////////////////
    
    mLog.write("Ready for Assembly and Launch Rail");
    mLog.write("-----------------------------------\n\n\n");
    
    float accelArray [numDataPointsChecked4Launch] = {};
    float accelAvg = 0;
    
    while(accelAvg < accelRoof*g0){
        response = mVN->readImuMeasurements();
        mLog.write(response);
        accelArray[0] = sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2)); // total accel vector
        accelAvg = calcArrayAverage(accelArray, numDataPointsChecked4Launch);
        
        for (int i = 0; i < numDataPointsChecked4Launch; ++i) { // shift array values down
            accelArray[i+1] = accelArray[i];
        }
    }
    
    mLog.write("Average acceleration exceeded " + to_string(accelRoof*g0) + " m/s^2 over " + to_string(numDataPointsChecked4Launch) + " data points");
    mLog.writeDelim("Rocket Has Launched");
    mLog.write("Waiting for Motor Burn Time");
    
    //activeSleep(tBurn, mVN, response, mLog); //NOT WORKING ATM
    
    int sleepTime = tBurn;
    double currentTime = getCurrentTime();
    double endTime = sleepTime + getCurrentTime();
    while (currentTime < endTime){
        response = mVN->readImuMeasurements();
        mLog.write(response);
        currentTime = getCurrentTime();
    }
    
    float zCurrent = 0;
    
    mLog.writeTime("Actively Checking Altitude");
    while (zCurrent < zDeploy) {
        response = mVN->readImuMeasurements();
        mLog.write(response);
        zCurrent = pressure2Altitude(T0, P0, g0, response.pressure);
        //mLog.write(to_string(zCurrent));
    }
    
    mLog.writeDelim("Deployment Altitude Eached");
    moveServoPair(servoPinN, servoPinS, airfoilTiltAngle); //CHECK IF NEED TO CONTINOUSLY "MOVE" SERVOS TO DEPLOY ANGLE
    mLog.writeTime("Airfoils Deployed");
    
    /* C O A S T I N G  S T A G E*//////////////////////////////////////
    
    float zCurrentArray [numDataPointsChecked4Apogee] = {};
    float maxAltitude = 0;
    int samplesSinceMaxHasChanged = 0;
    
    mLog.write("Looking for Apogee");
    
    // loop runs until we havent hit a new max altitude for numDataPointsChecked
    // i.e. we are not going up anymore
    while(samplesSinceMaxHasChanged < numDataPointsChecked4Apogee) {
        response = mVN->readImuMeasurements();
        mLog.write(response);
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
    
    mLog.write("Altitude has not reached a new max for " + to_string(numDataPointsChecked4Apogee) + " samples... deploying second pair of airfoils.");
    mLog.writeDelim("Apogee Detected");
    
    moveServoPair(servoPinN,servoPinS,12); 
    mLog.write("Second Pair of Airfoils Deployed");
    
    /* D E S C E N T  S T A G E *///////////////////////////////////////
    // all that needs to happen here is data keeps being saved and foils are kept at deploy angle
    //ADD END CONDITION
    
    zCurrentArray [numDataPointsChecked4Apogee] = {};
    float minAltitude = 1000000;
    int samplesSinceMinHasChanged = 0;
    
    while (samplesSinceMinHasChanged < numDataPointsChecked4Landing){
        response = mVN->readImuMeasurements();
        mLog.write(response);
        zCurrent = pressure2Altitude(T0, P0, g0, response.pressure);
        zCurrentArray[0] = zCurrent;
        
        if (zCurrent < minAltitude){
            minAltitude = zCurrent;
            samplesSinceMinHasChanged = 0;
        } else{
            ++samplesSinceMinHasChanged;
        }
        
        // shift array values
        for (int i = 0; i < numDataPointsChecked4Apogee; ++i) {
            zCurrentArray[i+1] = zCurrentArray[i];
        }
    }
        
    mLog.write("Altitude has not reached a new min for " + to_string(numDataPointsChecked4Apogee) + " samples... ending program.");
    mLog.writeDelim("Landing Detected");
    mLog.write("\n");
    //add ag check, time check
    
    if (IMU_ACTIVE){
        //mVN.disconnect();
        delete mVN;
        mLog.write("IMU: Disconnected");
	}
    // delete mVN; // delete only needed for pointers
    gpioTerminate();
    
    mLog.writeTime("\nEND PROGRAM");
    
    return 0;
}