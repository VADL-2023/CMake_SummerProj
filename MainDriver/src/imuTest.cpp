#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <ctime>
#include "ezasyncdata.h"


int main(){
    //Initialize pigpio (for servos)
    gpioInitialise();
    
    // IMU Connection and Configuration
    VnSensor* mVN = new VnSensor();
    
    std::cout << "IMU Connecting" << std::endl;
    mVN->connect(IMU_PORT,IMU_BAUD_RATE);
    if (!mVN->isConnected()){
        throw "IMU Failed to Connect";
    }else{
        std::cout << "IMU Connected\n" << std::endl;
    }
    EzAsyncData ez(mVN);

    ImuMeasurementsRegister response1;
    time_t t_start1, t_end1;
    time_t t_start2, t_end2;
    //Clock 100 measurements(using simplest Library method)
    time(&t_start1);
    for (int i = 0; i < 100; ++i){
        response1 = mVN->readImuMeasurements();
        //float accel = response.accel[2];
        //std::cout << "Accel Z: " << accel << std::endl;        
    }
    time(&t_end1);

    
    CompositeData response2;
    //Clock 100 measurements(using streamlined method)
    time(&t_start2);
    for (int i = 0; i < 100; ++i){
        response2 = ez.getNextData(1000);
        //float accel = response.accel[2];
        //std::cout << "Accel Z: " << accel << std::endl;        
    }
    time(&t_end2);
    
    //Print Results
    double elapsed_1  = double(t_end1 - t_start1);
    double elapsed_2  = double(t_end2 - t_start2);
    std::cout << "Time taken by original method: " << elapsed_1 << " sec" <<std::endl;
    std::cout << "Original frequency: " << 100/elapsed_1 << " Hz\n"<< std::endl;
    std::cout << "Time taken by streamlined method: " << elapsed_2 << " sec" << std::endl;
    std::cout << "Original frequency: " << 100/elapsed_2 << " Hz\n"<< std::endl;
    
    if (IMU_ACTIVE){
    std::cout << "IMU: Disconnecting" << std::endl;

    mVN->disconnect();

    std::cout << "IMU: Disconnected" << std::endl;
	}
    
    delete mVN;

	return 0;
}
