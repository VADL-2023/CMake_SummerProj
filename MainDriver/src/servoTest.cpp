#include "sensors.h"
#include "config_IMU.hpp"
#include "pigpio.h"
#include <stddef.h>

int main(){
    std::cout << "Initialize GPIO Returned: " << gpioInitialise() << std::endl;
        
    
    uint8_t servoPin = 18;
    // Initialize GPIO
    //gpioSetMode(servoPin, PI_OUTPUT);
    //gpioSleep(0,1,0);
    
    gpioServo(servoPin,1000);
    std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin) << std::endl;
    gpioSleep(0,2,0);
    gpioServo(servoPin,2000);
    std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin) << std::endl;
    gpioSleep(0,2,0);
    gpioServo(servoPin,1501);
    std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin) << std::endl;
    gpioSleep(0,2,0);
    gpioServo(servoPin,1000);
    std::cout << "Pulse: " << gpioGetServoPulsewidth(servoPin) << std::endl;
    gpioSleep(0,2,0);
    

/*
        for (int j = 0; j < 250; ++j){
        gpioWrite(servoPin,1);
        gpioSleep(0, 0, 1000);
        gpioWrite(servoPin,0); 
        gpioSleep(0, 0, 20000-1000);
        }
        
        for (int j = 0; j < 250; ++j){
        gpioWrite(servoPin,1);
        gpioSleep(0, 0, 2000);
        gpioWrite(servoPin,0); 
        gpioSleep(0, 0, 20000-2000);
        }
    */
    
    gpioTerminate();

	return 0;
}
