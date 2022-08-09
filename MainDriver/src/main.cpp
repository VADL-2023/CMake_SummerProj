#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <stddef.h>
#include <unistd.h> // has sleep() function
#include <math.h>

// conversion factors
float ft2m = 0.3048; // [m/ft]
float m2ft = 1/ft2m; // [ft/m]
float C2K = 273.15; // Celsius to Kelvin

// flight parameters
uint8_t finTiltAngle = 12; // [deg] fixed tilt angle for airfoil activation 
float h0 = 522*ft2m; // [m] elevation of launch site relative to sea level
float R = 287; // [J/kg/K] universal gas constant
float B = 0.0065; // [K/m] temperature lapse rate

// servo parameters
uint16_t pulseMin = 500; // [usecs] pulse width to send servo to one end of motion range
uint16_t pulseMax = 2500; // [usecs] pulse width to send servo to other end of motion range
uint8_t servoRange = 180; // [deg] possible range of motion of servo

// calibration parameters
uint16_t numSampleReadings = 25; // amount of samples taken and averaged to find ground P and T
float servoTestWaitTime = 1; // [s] amount of time between servo movement tests

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
void moveServoPair(uint8_t pin1, uint8_t pin2, float angle){
    float pulseWidth1 = convertAngle2PulseWidth(angle);
    float pulseWidth2 = convertAngle2PulseWidth(-angle);
    
    gpioServo(pin1, pulseWidth1);
    gpioServo(pin2, pulseWidth2);
}

// moves a single servo to specified angle [deg]
void moveServo(uint8_t pin, float angle) {
    float pulseWidth = convertAngle2PulseWidth(angle);
    gpioServo(pin,pulseWidth);
}

// given pin and angle, activates test protocol for a single servo
void testServo(uint8_t pin, float angle) {
    moveServo(pin, angle);
    gpioSleep(0,servoTestWaitTime,0);
    moveServo(pin, -angle);
    gpioSleep(0,servoTestWaitTime,0);
    moveServo(pin, 0); // end by moving servo to middle
    gpioSleep(0,servoTestWaitTime,0);
}

// given T0 [K], P0 [kPa], g0 [m/s^2], P [kPa], returns altitude relative to baseline
// 0 indicates baseline measurement, R and B are constants
float pressure2Altitude(float T0, float P0, float g0, float P) {
    return h0 + T0/B*(pow(P/P0,-R*B/g0) - 1);
}
    
int main(){
    /* P R E - F L I G H T  S T A G E*/
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
        std::cout << "IMU Connecting" << std::endl;
        mVN->connect(IMU_PORT,IMU_BAUD_RATE);
        if (!mVN->isConnected()){
            throw "IMU Failed to Connect";
        }else{
            std::cout << "IMU Connected" << std::endl;
        }
        
        // test all 4 servos
        std::cout << "Testing Servo Activation" << std::endl;
        sleep(2);
        
        std::cout << "Testing North..." << std::endl;
        testServo(servoPinN,finTiltAngle);
        std::cout << "Testing East..." << std::endl;
        testServo(servoPinE,finTiltAngle);
        std::cout << "Testing South..." << std::endl;
        testServo(servoPinS,finTiltAngle);
        std::cout << "Testing West..." << std::endl;
        testServo(servoPinW,finTiltAngle);
        
        // calibrate ground level pressure and temperature
        ImuMeasurementsRegister response;
        pressureSum = 0;
        tempSum = 0;
        gravSum = 0;
        
        std::cout << "Calibrating Baseline Parameters" << std::endl;
        
        for (int i = 0; i < numSampleReadings; ++i){
            response = mVN->readImuMeasurements();
            pressureSum += response.pressure;
            tempSum += response.temp;
            gravSum += sqrt(pow(response.accel[0],2) + pow(response.accel[1],2) + pow(response.accel[2],2));
        }
        P0 = pressureSum/numSampleReadings;
        T0 = tempSum/numSampleReadings + C2K;
        g0 = gravSum/numSampleReadings;
        
        std::cout << "Calibrated Temperature: " << T0 << " K" << std::endl;
        std::cout << "Calibrated Pressure: " << P0 << " kPa" << std::endl;
        std::cout << "Calibrated Gravity: " << g0 << " m/s^2" << std::endl;
        std::cout << "Are we a GO for flight?" << std::endl;
        std::cin >> go; // get user input
        
        // if no-go, undo initializations so we can try again
        if(go != "GO") {
            if (IMU_ACTIVE){
                mVN->disconnect();
                std::cout << "IMU Disconnected" << std::endl;
            }
            delete mVN;
            gpioTerminate();
            std::cout << "GPIO Terminated" << std::endl;
        } else { // if go, disconnect pointer so normal object can be created
            if (IMU_ACTIVE){
                mVN->disconnect();
                std::cout << "IMU Disconnected" << std::endl;
            }
            
        }
        
    }

    std::cout << "Pre-Flight Stage Passed" << std::endl;
    
    /* L A U N C H  S T A G E */
    std::cout << "Ready for Assembly and Launch Rail" << std::endl;
    
    VnSensor mVN;
    mVN.connect(IMU_PORT,IMU_BAUD_RATE);
    
    ImuMeasurementsRegister response;
    for (int i = 0; i < 100; ++i){
        response = mVN.readImuMeasurements(); //Actually dive into implementation so we're not sending the same command over and over
        float P = response.pressure;
        float z = pressure2Altitude(T0,P0,g0,P)*m2ft;
        
        std::cout << "Pressure: " << P << " kPa" << std::endl;
        std::cout << "Altitude: " << z << " ft" << std::endl;
    }
        
    // move servos
    moveServoPair(servoPinN,servoPinS,finTiltAngle);
    gpioSleep(0,1,0);
    moveServoPair(servoPinN,servoPinS,-finTiltAngle);
    gpioSleep(0,1,0);
    moveServoPair(servoPinN,servoPinS,finTiltAngle);
    
    if (IMU_ACTIVE){
        //mVN->disconnect();
        //std::cout << "IMU: Disconnected" << std::endl;
	}
    
    // delete mVN;
    gpioTerminate();
    
    return 0;
} // end main
    
     
    // HERE LIES CODE CEMETARY OF COMMENTED OUT CODE
    /*
    if (IMU_ACTIVE){
    std::cout << "IMU: Disconnecting" << std::endl;

    //mVN.unregisterAsyncPacketReceivedHandler();
    mVN.disconnect();

    std::cout << "IMU: Disconnected" << std::endl;
	}

    gpioServo(servoPin, 1500);
    gpioSleep(0,3,0);

	return 0; */
    
        //bool prevDir = false;
    //bool curDir = false;
    /*
    ImuMeasurementsRegister response;
    for (int i = 0; i < 100; ++i){
        response = mVN.readImuMeasurements(); //Actually dive into implementation so we're not sending the same command over and over
        float pres = response.accel[2];
        std::cout << "Temp: " << pres << std::endl;
        curDir = (pres > 0 ? true : false);
        if (curDir != prevDir){
            std::cout << "Direction change, moving servo" << std::endl;
            moveServo(curDir);
            prevDir = curDir;
        }*/

