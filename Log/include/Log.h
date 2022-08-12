#include "sensors.h"
#include "registers.h"
#include <fstream>

#include <chrono> 
#include <iostream>
#include <sys/time.h>
#include <ctime>

#ifndef MYPROJECT_NEWLOG_H
#define MYPROJECT_NEWLOG_H

class Log{    

    double startTime;
    double currentTime;

private:
    std::ofstream mFlightLog; // Flight Data Log File, relative to current directory

    std::ofstream mProgLog; // Program Output Log File, relative to current directory

    vn::sensors::VnSensor* mIMU;    

public:

    Log(std::string flightFilename, std::string programFilename, vn::sensors::VnSensor* imu, double sTime);

    ~Log();

    void write(vn::sensors::ImuMeasurementsRegister& data);
    
    void write(std::string outputString);

    void writeTime(std::string outputString);
    
    double elapsedTime();

};

#endif //MYPROJECT_NEWLOG_H
