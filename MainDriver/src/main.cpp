#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
uint8_t servoPin = 18;

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
        float accel = response.accel[2];
        std::cout << "Accel Z: " << accel << std::endl;
        curDir = (accel > 0 ? true : false);
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


	return 0;
}
