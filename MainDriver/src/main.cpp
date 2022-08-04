#include "foo.h"
#include "sensors.h"
#include "config_IMU.hpp"
#include "IMU.hpp"
#include "pigpio.h"

int main(){
    // IMU Connection and Configuration
    IMU mIMU;
    for (int i = 0; i < 10; ++i){
    mIMU.receive();
    std::cout << "Pressure: " << mIMU.pres << std::endl;
    }


	return 0;
}
