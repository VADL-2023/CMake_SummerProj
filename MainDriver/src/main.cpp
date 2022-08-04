#include "foo.h"
#include "sensors.h"
#include "config.hpp"
#include "IMU.hpp"

int main(){
    // IMU Connection and Configuration
    IMU mIMU;
    myVN.connect(IMU_PORT, IMU_BAUD_RATE);
	foo();
	//std::cout << "Got Here"<<std::endl;
	return 0;
}
