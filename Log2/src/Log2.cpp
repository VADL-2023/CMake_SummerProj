#include "../include/Log2.h"

Log2::Log2(std::ofstream &flight, std::ofstream &program, vn::sensors::VnSensor* imu):
mIMU(imu)
{
    mFlightLog = flight; 
    mProgLog = program;
    
    time(&startTime);
    if (!(mFlightLog.is_open() && mProgLog.is_open())){
        std::cout << "Error opening file streams";
    }else {
        mFlightLog
                << "Time, Mag X, Mag Y, Mag Z, Accel X, Accel Y, Accel Z, Y, P, R, Temp (C), Pressure (kPa)\n";
        mProgLog << "Successfully opened both file output streams \n";
    }
}

Log2::~Log2() {
    mProgLog << "In Destructor of Log2 object, closing fstreams, setting pointer to null\n";
    mFlightLog.close();
    mProgLog.close();
    mIMU = nullptr;
}
// May have to add 'vn::sensors::' before ImuMeasurementsRegister, hopefully not
void Log2::write(vn::sensors::ImuMeasurementsRegister& data){
    time_t curTime;
    time(&curTime);

    char* buf[256];
    sprintf(buf, "%g, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
            difftime(startTime, curTime), data.mag[0], data.mag[1], data.mag[2], data.accel[0], data.accel[1], data.accel[2], data.gyro[0], data.gyro[1], data.gyro[2]);
    mFlightLog << buf;
}

void Log2::write(std::string& outputString){
    mProgLog << outputString << '\n';
}
