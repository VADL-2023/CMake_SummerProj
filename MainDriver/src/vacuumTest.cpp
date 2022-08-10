#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <ctime>
#include "ezasyncdata.h"
#include "Log2.h"

int main(){
    //Initialize pigpio (for servos)
    gpioInitialise();
    
    // IMU Connection and Configuration
    VnSensor* mVN = new VnSensor();
    Log2 mLog("flightLogData", "programLogData", mVN);
    mLog.write("POG BABY POG");
    
    std::cout << "IMU Connecting" << std::endl;
    mVN->connect(IMU_PORT,IMU_BAUD_RATE);
    if (!mVN->isConnected()){
        throw "IMU Failed to Connect";
    }else{
        std::cout << "IMU Connected\n" << std::endl;
    }

    ImuMeasurementsRegister response1;
    time_t t_start1, t_end1;
    time_t t_start2, t_end2;
    //Clock 100 measurements(using simplest Library method)
    time(&t_start1);
    for (int i = 0; i < 100; ++i){
        response1 = mVN->readImuMeasurements();
        //pressure = response1.pressure;
        //std::cout << "Accel Z: " << pressure << std::endl;   
        mLog.write(response1);     
    }
    time(&t_end1);
    
    //Print Results
    double elapsed_1  = double(t_end1 - t_start1);
    std::cout << "Time taken by original method: " << elapsed_1 << " sec" <<std::endl;
    std::cout << "Original frequency: " << 100/elapsed_1 << " Hz\n"<< std::endl;
    
    if (IMU_ACTIVE){
    std::cout << "IMU: Disconnecting" << std::endl;

    mVN->disconnect();

    std::cout << "IMU: Disconnected" << std::endl;
    std::cout << "POG" << std::endl;
	}
    
    delete mVN;

	return 0;
}
