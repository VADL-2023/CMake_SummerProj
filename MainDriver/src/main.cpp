#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"

// define 4 servo pins
uint8_t servoPin = 18;
//uint8_t servoPin2 = 18;
//uint8_t servoPin3 = 23;
//uint8_t servoPin4 = 23;

void moveServo(bool dir){
    if (dir){
        gpioServo(servoPin,1000);
        std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin) << std::endl;
    }else{
        gpioServo(servoPin,2000);
        std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin) << std::endl;
    }
}

int main(){
    // IMU Connection and Configuration
    gpioInitialise();
    VnSensor mVN;
    std::cout << "IMU Connecting" << std::endl;
    mVN.connect(IMU_PORT,IMU_BAUD_RATE);
    if (!mVN.isConnected()){
        throw "IMU Failed to Connect";
    }else{
        std::cout << "IMU Connected" << std::endl;
    }
    
    
    bool prevDir = false;
    bool curDir = false;
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
        }
    }
    
     
    
    if (IMU_ACTIVE){
    std::cout << "IMU: Disconnecting" << std::endl;

    //mVN.unregisterAsyncPacketReceivedHandler();
    mVN.disconnect();

    std::cout << "IMU: Disconnected" << std::endl;
	}

    gpioServo(servoPin, 1500);
    gpioSleep(0,3,0);

	return 0;
}
