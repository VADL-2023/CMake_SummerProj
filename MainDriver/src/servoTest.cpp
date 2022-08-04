#include "foo.h"
#include "sensors.h"
#include "config_IMU.hpp"
#include "IMU.hpp"
#include "LOG.hpp"
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
    response = mVN.readImuMeasurements();
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
