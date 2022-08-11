#include "../include/Log2.h"

Log2::Log2(std::string flightFilename, std::string programFilename, vn::sensors::VnSensor* imu):
mIMU(imu)
{
    mFlightLog.open(flightFilename);
    mProgLog.open(programFilename);
    
    startClock = std::clock();
    auto mTimeStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    
    //time_t t_start, t_end;
    //time(&t_start);
    
    mProgLog << "Start Clock" <<startClock << std::endl;
    if (!(mFlightLog.is_open() && mProgLog.is_open())){
        mProgLog << "Error opening file streams";
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
void Log2::write(vn::sensors::ImuMeasurementsRegister& data, double elapsedTime){
    clock_t currentClock = std::clock();
    //std::cout << "End Clock" <<currentClock << std::endl;
    //std::cout << "Clock" <<currentClock << std::endl;
    
    //auto mTimeEnd = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    //int elapsedTime  = int(mTimeEnd - mTimeStart);
    //std::cout <<elapsedTime <<std::endl;
    char buf[256];
    sprintf(buf, "%g, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
            elapsedTime, data.mag[0], data.mag[1], data.mag[2], data.accel[0], data.accel[1], data.accel[2], data.gyro[0], data.gyro[1], data.gyro[2], data.temp, data.pressure);
    mFlightLog << buf;
    //133260.4
}

void Log2::write(std::string outputString){
    mProgLog << outputString << '\n';
}
