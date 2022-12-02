#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <stddef.h>
#include <unistd.h>
#include <math.h>
#include <algorithm> 
#include <iostream>
#include "Log.h"

// conversion factors
float ft2m = 0.3048; // [m/ft]
float m2ft = 1/ft2m; // [ft/m]
float C2K = 273.15; // Celsius to Kelvin
float km2m = 0.001; // [km/m]

// constants
float R = 287; // [kg/JK] universal gas constant
float B = 6.5*km2m; // [K/m] variation of temperature within the troposphere
bool restart = false; // tells the program whether or not we NO-GOed
bool failedIMU = false; // whether or not IMU has failed
bool timeToDeployOverride = false; // whether or not to go to lastest deployment mode if time exceeds timeToDeploy

// fixed flight parameters
uint8_t airfoilTiltAngle = 12; // [deg] fixed tilt angle for airfoil activation 
float tBurn = 1.6; // [s] motor burn time
float samplingFrequency = 20; // [Hz] how fast does the IMU sample data

// possibly variable flight parameters (stuff we might change)
float accelRoof = 1.5; // how many g's does the program need to see in order for launch to be detected
int numDataPointsChecked4Launch = 8; // how many acceleration points are averaged to see if data set is over accelRoof
int numDataPointsChecked4Apogee = 10; // how many altitude points must a new max not be found for apogee to be declared
int numDataPointsChecked4Landing = 10*samplingFrequency; // how many altitude points must a new min not be found for landing to be declared
float zDeployPrimary = 1050*ft2m; // [m] altitude at which airfoils will deploy AGL in pitch configuration
float zDeploySecondary = 1500*ft2m; // [m] altitude at which airfoils will deploy AGL in drag configuration 
bool dualDeployEvents = true; // whether or not dual deployment events will occur; false means only primary deployment will occur
bool pitchFirst = true; // whether or not to do pitch config first if doing dual deployment
bool servoTest = false; // whether or not to test actuation range of servos during GO/NOGO
int maxFlightTime = 180; // [s] max allowable flight time, if exceeded program ends
int timeToDeploy = 10; // [s] deploy servos after this amount of time from launch detection (5s after launch = 900ft)

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

// moves a pair of servos in opposite CW/CCW directions (but same linear direction) 
// by the same amount (you are outside of the rocket facing radially inward) if pitchConfig == true (pitch configuration)
// else moves a pair of servos in same CW/CCW directions (different linear direction) if pitchConfig == false (drag configuration)
void moveServoPair(int8_t pin1, uint8_t pin2, float angle, bool pitchConfig){
    if(pitchConfig){
        // pitch config
        
        float pulseWidth1 = convertAngle2PulseWidth(angle);
        float pulseWidth2 = convertAngle2PulseWidth(-angle);
        
        gpioServo(pin1, pulseWidth1);
        gpioServo(pin2, pulseWidth2);
    } else{
        // drag config
        
        float pulseWidth1 = convertAngle2PulseWidth(angle);
        float pulseWidth2 = convertAngle2PulseWidth(angle);
        
        gpioServo(pin1, pulseWidth1);
        gpioServo(pin2, pulseWidth2);
    }
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

// checks if flight has gone on for too long, if so returns true to end program / deploy airfoils
bool timeFlight(double launchTime, double triggerTime){
    if (getCurrentTime() - launchTime > triggerTime*1000){
        return true;
    } else {
        return false;
    }
}

// disconnects IMU and gpio, returns true if IMU was active but was successfuly disconnected
bool terminateConnections(VnSensor* imu){
    bool out;
    if (IMU_ACTIVE){
        out = true;
        imu->disconnect();
	} else {
        out = false;
    }
    
    gpioTerminate();
    return out;
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
    double startTime; //milliseconds since epoch
    
    VnSensor* mVN; 
    ImuMeasurementsRegister response;
    
    startTime = getCurrentTime();
    Log mLog("AAC Reflight Flight Log TEST", "AAC Reflight Program Log TEST", mVN, startTime); // don't use special characters in filename
    
    mLog.write("Date: 12-2");
    mLog.write("Flight Name: VC Testings\n");
    mLog.write("Test Notes: Log Testings\n");
    mLog.write("Verify Critical Parameters: ");
    mLog.write("Max Flight Time: " + to_string(maxFlightTime) + " s");
    mLog.write("Max Time to Deploy: " + to_string(timeToDeploy) + " s");
    mLog.write("Primary Deployment Altitude: " + to_string(zDeployPrimary*m2ft) + " Feet AGL");
    mLog.write("Primary Deployment Altitude: " + to_string(zDeployPrimary) + " Meters AGL");
    mLog.write("Dual Deployment Events: " + to_string(dualDeployEvents));
    mLog.write("Pitch Configuration Is Primary: " + to_string(pitchFirst));
    mLog.write("Secondary Deployment Altitude: " + to_string(zDeploySecondary*m2ft) + " Feet AGL");
    mLog.write("Secondary Deployment Altitude: " + to_string(zDeploySecondary) + " Meters AGL");
    mLog.write("Deployment Angle: " + to_string(airfoilTiltAngle) + " Degrees");
    mLog.write("Motor Burn Time: " + to_string(tBurn) + " Seconds");
    mLog.write("Trigger Acceleration: " + to_string(accelRoof) + " g");
    mLog.write("Launch Detection Samples: " + to_string(numDataPointsChecked4Launch));
    mLog.write("Apogee Detection Samples: " + to_string(numDataPointsChecked4Apogee));
    mLog.write("Landing Detection Samples: " + to_string(numDataPointsChecked4Landing));
    mLog.write("-----------------------------------\n\n\n");
    sleep(5);
    
    // begin GO-NOGO Protocol
    string go = "NOGO";
    
    while (go != "GO") {
        
        //disconnect VN if we had a NOGO response
        if (restart){
            mVN->disconnect();
            restart = false;
        }
        
        // IMU connection and configuration
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
        
        // connect to IMU
        mLog.write("IMU Connecting");
        try{
            mVN->connect(IMU_PORT, IMU_BAUD_RATE);
            mLog.write("IMU Connected");
        } catch(std::exception){
            mLog.write("IMU failed to connect... restart program");
            return 0;
        }

            
        // test all 4 servos
        if(servoTest){
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
        } 
    
        // flush IMU data during init
        mLog.write("IMU Flushing");
        for (int i = 0; i < imuWait; ++i){
            try{
                response = mVN->readImuMeasurements();
                mLog.write(response);
            } catch(std::exception){
                mLog.write("IMU disconnected during flush... restart program");
                return 0;
            }
        }
        mLog.write("IMU Flushed");
    
        // calibrate ground level pressure and temperature
        pressureSum = 0;
        tempSum = 0;
        gravSum = 0;
        mLog.writeTime("Calibrating Baseline Parameters. Hold Still.");
        
        for (int i = 0; i < numSampleReadings; ++i){
            try{
                response = mVN->readImuMeasurements();
                mLog.write(response);
                pressureSum += response.pressure;
                tempSum += response.temp;
                gravSum += sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2));
            } catch(std::exception){
                mLog.write("IMU disconnected during calibration... restart program");
                return 0;
            }
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
        mLog.write("");
        
        if (go != "GO"){
            restart = true;
        }
    }
        
    mLog.writeTime("Pre-Flight Stage Completed");
    
    /* L A U N C H  S T A G E */////////////////////////////////////////
    
    mLog.write("Ready for Assembly and Launch Rail");
    
    float accelArray [numDataPointsChecked4Launch] = {0};
    float accelAvg = 0;
    int counter = 0;
    
    // launched detected when avg accel exceeds accelRoof*g0 for numDataPointsChecked4Launch
    while(accelAvg < accelRoof*g0){
        try{
            response = mVN->readImuMeasurements();
            mLog.write(response);
        } catch(std::exception){
            mLog.write("IMU disconnected while waiting for launch");
            mLog.writeTime("Ending Program");
            terminateConnections(mVN);
            return 0;
        } 
        
        accelArray[counter%numDataPointsChecked4Launch] = sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2)); // total accel vector
        accelAvg = calcArrayAverage(accelArray, numDataPointsChecked4Launch);
        ++counter;
    }
    
    mLog.write("Average acceleration exceeded " + to_string(accelRoof*g0) + " m/s^2 over " + to_string(numDataPointsChecked4Launch) + " data points");
    mLog.writeDelim("Rocket Has Launched");
    mLog.write("Waiting for Motor Burn Time");
    
    double launchTime = getCurrentTime();
    
    activeSleep(tBurn, mVN, response, mLog); //wait for motor burn to complete while still collecting IMU data
    
    float zCurrent = 0;
    
    // begin checking for first deployment altitude (pitching)
    mLog.writeTime("Actively Checking Altitude for first deployment");
    while (zCurrent < zDeployPrimary) {
        try{
            response = mVN->readImuMeasurements();
            mLog.write(response);
            zCurrent = pressure2Altitude(T0, P0, g0, response.pressure);
        } catch(std::exception){
            if(!failedIMU){
                mLog.write("IMU disconnected while waiting for deployment altitude... waiting for time override");
                failedIMU = true;
                zCurrent = 0; //only exit loop after timeToDeploy seconds have passed
            }
        } 
        
        // ensure program successfully exits if time exceeds limit\
        // NOTE: this seems unneeded as shouldn't timeToDeploy always trigger before this?
        if (timeFlight(launchTime, maxFlightTime)) {
            if(terminateConnections(mVN)){
                mLog.write("IMU: Disconnected");
            }
            mLog.writeTime("Ending program due to time overflow");
            return 0;
        }
        
        // if timeToDeploy seconds pass after launch detection, move on to next part of code
        // if there are dual deployment events, move to actuate in secondary condition only
        if (timeFlight(launchTime, timeToDeploy)){
            if (!dualDeployEvents){
                mLog.write("Deploying airfoils due to time override after " + to_string(timeToDeploy));
            } else{
                timeToDeployOverride = true;
            }
            break;
        }
    }
    
        // actuate servos if either we reached zDeployPrimary alt or if (there are not dual deployment events and
        // the time override occurred)
        if (zCurrent >= zDeployPrimary || (!dualDeployEvents && !timeToDeployOverride)){
            // deploy both pairs of airfoils at primary deployment alt in pitch configuration
            mLog.writeDelim("First Deployment Altitude Reached");
            moveServoPair(servoPinN, servoPinS, airfoilTiltAngle, pitchFirst);
            moveServoPair(servoPinE, servoPinW, airfoilTiltAngle, pitchFirst);
            mLog.writeTime("Airfoils Deployed in Pitch Configuration");
        }
    }
    
    // begin checking for second deployment altitude (drag configuration)
    // NOTE: Check logic for time override; Check logic for dualDeployEvents
    if(dualDeployEvents){
        mLog.writeTime("Actively Checking Altitude for second deployment");
        while (zCurrent < zDeploySecondary) {
            try{
                response = mVN->readImuMeasurements();
                mLog.write(response);
                zCurrent = pressure2Altitude(T0, P0, g0, response.pressure);
            } catch(std::exception){
                if(!failedIMU){
                    mLog.write("IMU disconnected while waiting for deployment altitude... waiting for time override");
                    failedIMU = true;
                    zCurrent = 0; //only exit loop after timeToDeploy seconds have passed
                }
            } 
            
            // ensure program successfully exits if time exceeds limit
            // NOTE: this seems unneeded as shouldn't timeToDeploy always trigger before this?
            if (timeFlight(launchTime, maxFlightTime)) {
                if(terminateConnections(mVN)){
                    mLog.write("IMU: Disconnected");
                }
                mLog.writeTime("Ending program due to time overflow");
                return 0;
            }
            
            // if timeToDeploy seconds pass after launch detection, move on to next part of code
            if (timeFlight(launchTime, timeToDeploy)){
                mLog.write("Deploying airfoils due to time override after " + to_string(timeToDeploy));
                timeToDeployOverride = true;
                break;
            }
        }
        
        // deploy both pairs of airfoils at secondary deployment alt in drag configuration
        mLog.writeDelim("Second Deployment Altitude Reached");
        moveServoPair(servoPinN, servoPinS, airfoilTiltAngle, !pitchFirst);
        moveServoPair(servoPinW, servoPinE, -airfoilTiltAngle, !pitchFirst);
        mLog.writeTime("Airfoils Deployed in Drag Configuration");
    }
    
    /* C O A S T I N G  S T A G E*//////////////////////////////////////
    
    float maxAltitude = 0;
    int samplesSinceMaxHasChanged = 0;
    
    mLog.write("Looking for Apogee");
    
    // apogee detected when a new max altitude has not been achieved for numDataPointsChecked4Apogee
    while(samplesSinceMaxHasChanged < numDataPointsChecked4Apogee) {
        try{
            response = mVN->readImuMeasurements();
            mLog.write(response);
            zCurrent = pressure2Altitude(T0, P0, g0, response.pressure);
            
            if (zCurrent >= maxAltitude) {
                maxAltitude = zCurrent;
                samplesSinceMaxHasChanged = 0;
            } else {
                ++samplesSinceMaxHasChanged;
            }
            
        } catch(std::exception){
            mLog.write("IMU disconnected while waiting for apogee");
            mLog.writeTime("Ending Program");
            terminateConnections(mVN);
            return 0;
        } 
        
        // ensure program successfully exits if time exceeds limit
        if (timeFlight(launchTime, maxFlightTime)) {
            if(terminateConnections(mVN)){
                mLog.write("IMU: Disconnected");
            }
            mLog.writeTime("Ending program due to time overflow");
            return 0;
        }
    }
    
    //deploy secondary pair of airfoils at apogee
    mLog.write("Altitude has not reached a new max for " + to_string(numDataPointsChecked4Apogee) + " samples... deploying second pair of airfoils.");
    mLog.writeDelim("Apogee Detected");
    
    /* D E S C E N T  S T A G E *///////////////////////////////////////

    float minAltitude = 1000000; 
    int samplesSinceMinHasChanged = 0;
    
    // landing detected when a new min altitude has not been achieved for numDataPointsChecked4Landing
    while (samplesSinceMinHasChanged < numDataPointsChecked4Landing){
        try{
            response = mVN->readImuMeasurements();
            mLog.write(response);
            zCurrent = pressure2Altitude(T0, P0, g0, response.pressure);
            
            if (zCurrent < minAltitude){
                minAltitude = zCurrent;
                samplesSinceMinHasChanged = 0;
            } else{
                ++samplesSinceMinHasChanged;
            }
            
        } catch(std::exception){
            mLog.write("IMU disconnected while waiting for landing");
            mLog.writeTime("Ending Program");
            terminateConnections(mVN);
            return 0;
        } 
        
        // ensure program successfully exits if time exceeds limit
        if (timeFlight(launchTime, maxFlightTime)) {
            if(terminateConnections(mVN)){
                mLog.write("IMU: Disconnected");
            }
            mLog.writeTime("Ending program due to time overflow");
            return 0;
        }
    }
        
    mLog.write("Altitude has not reached a new min for " + to_string(numDataPointsChecked4Landing) + " samples... ending program.");
    mLog.writeDelim("Landing Detected");
    
    if(terminateConnections(mVN)){
        mLog.write("IMU: Disconnected");
    }    
    
    mLog.writeTime("\nEND PROGRAM");

    return 0;
}
