
#include "sensors.h"
#include "registers.h"
#include <fstream>


#ifndef MYPROJECT_NEWLOG_H
#define MYPROJECT_NEWLOG_H

class Log2{
private:
    std::ofstream mFlightLog; // Flight Data Log File, relative to current directory

    std::ofstream mProgLog; // Program Output Log File, relative to current directory

    VnSensor* mIMU;

public:

    Log2(std::ofstream &flight, std::ofstream &program);

    // May have to add 'vn::sensors::' before ImuMeasurementsRegister, hopefully not
    void write(ImuMeasurementsRegister& data);

    void write(std::string& outputString);

};

#endif //MYPROJECT_NEWLOG_H
