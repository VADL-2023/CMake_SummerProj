#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <ctime>
#include "IMU.hpp"
#include "LOG.hpp"

int main(){
    //Initialize pigpio (for servos)
    gpioInitialise();
    
    IMU mVN;
    LOG mLog(mVN);
    
/*
    time_t t_start1, t_end1;
    //Clock 100 measurements(using simplest Library method)
    time(&t_start1);
    for (int i = 0; i < 100; ++i){
        newData = ez->getNextData(50);
        std::cout << (newData.hasPressure() ? newData.pressure() : 0) << std::endl;
        //float accel = response.accel[2];
        //std::cout << "Accel Z: " << accel << std::endl;        
    }
    time(&t_end1);

    //Print Results
    double elapsed_1  = double(t_end1 - t_start1);
    std::cout << "Time taken by original method: " << elapsed_1 << " sec" <<std::endl;
    std::cout << "Original frequency: " << 100/elapsed_1 << " Hz\n"<< std::endl;
    
*/

	return 0;
}
