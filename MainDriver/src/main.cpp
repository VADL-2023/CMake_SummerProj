#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"

int main(){
    // IMU Connection and Configuration
    VnSensor mVN;
    std::cout << "IMU Connecting" << std::endl;
    mVN.connect(IMU_PORT,IMU_BAUD_RATE);
    if (!mVN.isConnected()){
        throw "IMU Failed to Connect";
    }else{
        std::cout << "IMU Connected" << std::endl;
    }
    
    ImuMeasurementsRegister response;
    for (int i = 0; i < 40; ++i){
    response = mVN.readImuMeasurements(); //Actually dive into implementation so we're not sending the same command over and over
    //std::cout << "Pressure: " << response.pressure << std::endl;
    }
    
    if (IMU_ACTIVE){
    std::cout << "IMU: Disconnecting" << std::endl;

    //mVN.unregisterAsyncPacketReceivedHandler();
    mVN.disconnect();

    std::cout << "IMU: Disconnected" << std::endl;
	}


	return 0;
}
