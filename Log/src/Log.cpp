#include "../include/Log.h"

Log::Log(std::string flightFilename, std::string programFilename, vn::sensors::VnSensor* imu, double sTime):
mIMU(imu)
{
    //initialize variables
    this->startTime = sTime;
    this->savedParameters = false;
    this->delim = "$";
    
    //open flight and program data files
    mFlightLog.open(flightFilename);
    mProgLog.open(programFilename);

    if (!(mFlightLog.is_open() && mProgLog.is_open())){
        mProgLog << "Error opening file streams";
    }else {
        mFlightLog
                << "Time, MagX, MagY, MagZ, AccelX, AccelY, AccelZ, Yaw, Pitch, Roll, Temperature, Pressure, Altitude\n";
        mProgLog << "Start Time: " << std::__cxx11::to_string(startTime) << "\n";
        mProgLog << "Successfully opened both file output streams \n";
    }
}

// deletion of Log pointer, close flight and program data files
Log::~Log() {
    mProgLog << "In Destructor of Log2 object, closing fstreams, setting pointer to null\n";
    mProgLog << "End Time: ";
    mProgLog << std::__cxx11::to_string(startTime + elapsedTime()) << '\n';
    mFlightLog.close();
    mProgLog.close();
    mIMU = nullptr;
}

// write IMU data to flight data file, will provide calculated altitude if baseline parameters are saved
void Log::write(vn::sensors::ImuMeasurementsRegister& data){
    if (savedParameters){
        currentAlt = calcAlt(data);
    } else{
        currentAlt = -1;
    }
    
    char buf[256];
    sprintf(buf, "%g, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f,\n",
            elapsedTime(), data.mag[0], data.mag[1], data.mag[2], data.accel[0], data.accel[1], data.accel[2], data.gyro[0], data.gyro[1], data.gyro[2], data.temp, data.pressure, currentAlt);
    mFlightLog << buf;
}

// write string to program data file and to console
void Log::write(std::string outputString){
    std::cout << outputString << std::endl;
    mProgLog << outputString << '\n';
}

// write string to program data file with timestamp and to console
void Log::writeTime(std::string outputString){
    std::cout << outputString << std::endl;
    mProgLog << outputString + " (" + std::to_string(elapsedTime()) << ")\n";
    //mProgLog << elapsedTime() << '\n';
}

// write string to program data file with special delimiters and timestamp for MATLAB postprocessing, and to console
void Log::writeDelim(std::string outputString){
    std::cout << outputString << std::endl;
    mProgLog <<delim + outputString + " (" + std::to_string(elapsedTime()) + ")" + delim << '\n';
}

// calculate time in milliseconds since Log pointer was created
double Log::elapsedTime(){
    currentTime = double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    return currentTime - startTime;
}

//save baseline parameters to calculate altitude
void Log::saveBaselineParameters(float Rconst, float Bconst, float pressure, float temperature, float accelg){
    this->R = Rconst;
    this->B = Bconst;
    this->pressure0 = pressure;
    this->temperature0 = temperature;
    this->accelg0 = accelg;
    savedParameters = true;
}

//calculate altitude based on baseline parameters and current IMU pressure data
float Log::calcAlt(vn::sensors::ImuMeasurementsRegister& data){
    return temperature0/B*(pow(data.pressure/pressure0,-R*B/accelg0) - 1);
}
