#include "../include/Log.h"

Log::Log(std::string flightFilename, std::string programFilename, vn::sensors::VnSensor* imu, double sTime):
mIMU(imu)
{
    this->startTime = sTime;
    //currentTime = double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    
    mFlightLog.open(flightFilename);
    mProgLog.open(programFilename);

    if (!(mFlightLog.is_open() && mProgLog.is_open())){
        mProgLog << "Error opening file streams";
    }else {
        mFlightLog
                << "Time, Mag X, Mag Y, Mag Z, Accel X, Accel Y, Accel Z, Y, P, R, Temp (C), Pressure (kPa)\n";
        mProgLog << startTime << "\n";
        mProgLog << "Successfully opened both file output streams \n";
    }
}

Log::~Log() {
    mProgLog << "In Destructor of Log2 object, closing fstreams, setting pointer to null\n";
    mProgLog <<elapsedTime() << '\n';
    mFlightLog.close();
    mProgLog.close();
    mIMU = nullptr;
}
// May have to add 'vn::sensors::' before ImuMeasurementsRegister, hopefully not
void Log::write(vn::sensors::ImuMeasurementsRegister& data){
    
    char buf[256];
    sprintf(buf, "%g, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
            elapsedTime(), data.mag[0], data.mag[1], data.mag[2], data.accel[0], data.accel[1], data.accel[2], data.gyro[0], data.gyro[1], data.gyro[2], data.temp, data.pressure);
    mFlightLog << buf;
}

void Log::write(std::string outputString){
    std::cout << outputString << std::endl;
    mProgLog << outputString << '\n';
}

void Log::writeTime(std::string outputString){
    std::cout << outputString << std::endl;
    mProgLog << outputString << '\n';
    mProgLog << elapsedTime() << '\n';
}

double Log::elapsedTime(){
    currentTime = double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    return currentTime - startTime;
}
