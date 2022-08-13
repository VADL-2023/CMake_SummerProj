#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <ctime>
#include "ezasyncdata.h"
#include "Log.h"
#include <unistd.h> // has sleep() function

bool runServos = true;
bool NandS = true;

// calibration parameters
uint16_t numSampleReadings = 25; // amount of samples taken and averaged to find ground P and T
float servoTestTiltWaitTime = 1; // [s] amount of time between servo movement tests
float servoTestBeginWaitTime = 1; // [s] amount of time before servo tests begin

// conversion factors
float ft2m = 0.3048; // [m/ft]
float m2ft = 1/ft2m; // [ft/m]
float C2K = 273.15; // Celsius to Kelvin
float km2m = 0.001; // [km/m]

// constants
float R = 287; // [kg/J/K] universal gas constant
float B = 6.5*km2m; //[K/m] variation of temperature wit5V DC via USB-C connector (minimum 3A*)hin the troposphere

// servo parameters
uint16_t pulseMin = 500; // [usecs] pulse width to send servo to one end of motion range
uint16_t pulseMax = 2500; // [usecs] pulse width to send servo to other end of motion range
uint8_t servoRange = 180; // [deg] possible range of motion of servo

// define 4 servo pins
uint8_t servoPinN = 18;
uint8_t servoPinE = 12;
uint8_t servoPinS = 13;
uint8_t servoPinW = 19;

uint16_t nMeasurements = 30;
float targetAlt = 550*ft2m; //AGL (m)
//float targetAltMin = 500*ft2m; //AGL (m)
bool targetDetected = false;
bool apogeeReady = false;

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

// given T0 [K], P0 [kPa], g0 [m/s^2], P [kPa], returns altitude above ground level
// 0 indicates baseline measurement, R and B are constants
float pressure2Altitude(float T0, float P0, float g0, float P) {
    return T0/B*(pow(P/P0,-R*B/g0) - 1);
}



int main(){
    //Initialize pigpio (for servos)
    gpioInitialise();
    
    // initialize servo pins as output
    gpioSetMode(servoPinN, PI_OUTPUT);
    gpioSetMode(servoPinE, PI_OUTPUT);
    gpioSetMode(servoPinS, PI_OUTPUT);
    gpioSetMode(servoPinW, PI_OUTPUT);
    
    // IMU Connection and Configuration
    VnSensor* mVN = new VnSensor();
    ImuMeasurementsRegister response;
    
    float moveangle = 0;
    
    if(runServos){
        moveServoPair(servoPinN, servoPinS, moveangle);
        sleep(1);
        moveServoPair(servoPinE, servoPinW, moveangle);
        sleep(1);
    }
    
    
    float pressureSum;
    float tempSum;
    float gravSum;
    float P0; // pressure at ground level
    float T0; // temperature at ground level
    float g0; // gravity at ground level
    
    /*
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
    */
    
    //sleep(5);
    
    //response = mVN->readImuMeasurements();
    //P0 = response.pressure;
    //T0 = response.temp;
    //g0 = sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2));

    auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    //Initialize Log object to save data
    Log mLog("VT Flight Data 9", "VT Program Data 9", mVN, double(startTime));
    
    P0 = 99.73;
    T0 = 30.7;
    g0 = 9.5;
    
    mLog.saveBaselineParameters(R, B, P0, T0, g0);
    
    mLog.write("Date: 8/12");
    mLog.write("Test Number: 9");
    mLog.write("Number of samples: ");
    mLog.write(to_string(nMeasurements));
    mLog.write("Expected run time (us): ");
    mLog.write(to_string(nMeasurements/.02));
    mLog.write("Deployment altitude: ");
    mLog.write(to_string(targetAlt));
    mLog.write("-----------------------------------");
    mLog.write("IMU Connecting");

    mVN->connect(IMU_PORT,IMU_BAUD_RATE);
    if (!mVN->isConnected()){
        throw "IMU Failed to Connect";
    }else{
        mLog.write("IMU Connected");
    }
    
    time_t t_start1, t_end1;
    time(&t_start1);

    auto mTimeStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    for (int i = 0; i < nMeasurements; ++i){
        response = mVN->readImuMeasurements();
     
     /*   
        if (i % 5){
            mLog.writeDelim("TEST DELIM");
        } else {
            mLog.write("IGNORE");
        }
       */
        
        //mLog.write(to_string(pressure2Altitude(T0, P0, g0, response.pressure)));
        
        if (pressure2Altitude(T0, P0, g0, response.pressure) > targetAlt && !targetDetected){
            targetDetected = true;
            apogeeReady = true;
            mLog.writeTime("Actuate Servos!");
            //mLog.write(to_string(response.pressure));
            //mLog.write(to_string(pressure2Altitude(T0, P0, g0, response.pressure)));
            
            if (runServos) {
                if (NandS){
                    mLog.writeTime("Starting to Actuate (N and S)");
                    moveServoPair(servoPinN, servoPinS, 45);
                    mLog.writeTime("Finished Actuating");
                } else {
                    
                    mLog.writeTime("Starting to Actuate (E and W)");
                    moveServoPair(servoPinE, servoPinW, 15);
                    mLog.writeTime("Finished Actuating");
                }
            }
         } else if(apogeeReady){
            apogeeReady = false;
         
            int numDataPointsChecked4Apogee = 15;
            float zCurrent = 0;
            float zCurrentArray [numDataPointsChecked4Apogee] = {};
            float maxAltitude = 0;
            int samplesSinceMaxHasChanged = 0;
    
            // loop runs until we havent hit a new max altitude for numDataPointsChecked
            // i.e. we are not going up anymore
            while(samplesSinceMaxHasChanged < numDataPointsChecked4Apogee) {
                response = mVN->readImuMeasurements();
                zCurrent = pressure2Altitude(T0,P0,g0,response.pressure);
                zCurrentArray[0] = zCurrent;
        
                if (zCurrent >= maxAltitude) {
                    maxAltitude = zCurrent;
                    samplesSinceMaxHasChanged = 0;
                } else {
                    ++samplesSinceMaxHasChanged;
                }
                
                mLog.write(to_string(zCurrent));
                mLog.write(to_string(samplesSinceMaxHasChanged));
        
                // shift array values
                for (int i = 0; i < numDataPointsChecked4Apogee; ++i) {
                    zCurrentArray[i+1] = zCurrentArray[i];
                }
    }
    
            mLog.writeTime("Altitude has not reached a new max for ");
            mLog.write(to_string(numDataPointsChecked4Apogee));
            mLog.write(" samples... retracting fins now.");
    
            moveServoPair(servoPinE,servoPinW, 45);
            mLog.writeTime("Fins Undeployed");
            }
         
         
         
         
         //if (response1.pressure > (targetPressure + .5) && targetDetected){
            //targetDetected = false;
            //mLog.write("Descending back down!");
            //mLog.write(to_string(response1.pressure));
         //}
         
        //auto mTimeEnd = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //double elapsedTime = double(mTimeEnd - mTimeStart);
        mLog.write(response);     
    }
    //time_t time(&t_end1);
    
    //Log Results
    //double elapsed_1 = double(t_end1 - t_start1);
    
    //mLog.write("Time taken (s): "); 
    //mLog.write(to_string(elapsed_1));
    //mLog.write("Frequency: ");
    //mLog.write(to_string(nMeasurements/elapsed_1));
    
    if (IMU_ACTIVE){
    mLog.write("IMU: Disconnecting");

    mVN->disconnect();

    mLog.write("IMU: Disconnected");
    }
    
    delete mVN;
    mLog.writeTime("END");
	return 0;
}



