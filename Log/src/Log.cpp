#include "../include/Log.h"

Log::Log(std::string fFilename, std::string pFilename, vn::sensors::VnSensor* imu, double sTime):
mIMU(imu)
{
    // initialize variables
    this->startTime = sTime;
    this->currTime = sTime;
    this->lastTime = 0;
    this->sampleNumber = 1;
    this->savedParameters = false;
    this->delim = "$";
    std::string timestring = std::__cxx11::to_string(startTime);
    timestring = timestring.substr(0, timestring.find('.'));
    
    this->flightFilename = fFilename + "_" + timestring + ".txt";
    this->programFilename = pFilename + "_" + timestring + ".txt";
    //std::cout << "Initial name: " << fFilename << std::endl;
    
    // catch errors involving special characters in filenames rip AAC madien flight 8/20 D:
    for (size_t i = 0; i < numSpecialCharacters; i++){
        if(fFilename.find(specialCharacters[i]) != std::string::npos || programFilename.find(specialCharacters[i]) != std::string::npos){
            throw std::invalid_argument("Remove special characters from log filenames");
        }
    }
    
    // open flight and program data files
    mFlightLog.open(flightFilename);
    mProgLog.open(programFilename);

    if (!(mFlightLog.is_open() && mProgLog.is_open())){
        mProgLog << "Error opening file streams";
    }else {
        mFlightLog
                << "Time\t MagX\t MagY\t MagZ\t AccelX\t AccelY\t AccelZ\t Yaw\t Pitch\t Roll\t Temperature\t Pressure\t Altitude\n";
        mProgLog << "START TIME: " << timestring << "\n";
    }
    
    // close flight and program data files
    mFlightLog.close();
    mProgLog.close();
}

// append last curent data, deletion of Log pointer, close flight and program data files
Log::~Log() {
    // reopen files
    mFlightLog.open(flightFilename, std::ofstream::app);
    mProgLog.open(programFilename, std::ofstream::app);
    
    // append current data
    mFlightLog << currentFlightData;
    mProgLog << currentProgData;
    
    // Note ending time
    mProgLog << "END TIME: ";
    mProgLog << std::__cxx11::to_string(startTime + elapsedTime()) << '\n';
    mProgLog << "END LOG";
    
    // close files and IMU pointer
    mFlightLog.close();
    mProgLog.close();
    mIMU = nullptr;
}

// write IMU data to flight data file, will provide calculated altitude if baseline parameters are saved
void Log::write(vn::sensors::ImuMeasurementsRegister& data){
    if (savedParameters){
        currentAlt = calcAlt(data);
    } else{
        currentAlt = -99999;
    }
    
    ++sampleNumber;
    
    // save and reopen file after 1000 ms
    currTime = elapsedTime();
    if ((currTime - lastTime) > 1000){
            //reopen files
            mFlightLog.open(flightFilename, std::ofstream::app);
            mProgLog.open(programFilename, std::ofstream::app);
            
            // appened current data
            mFlightLog << currentFlightData;
            mProgLog << currentProgData;
            
            // close files
            mFlightLog.close();
            mProgLog.close();
            
            // reset variables
            currentFlightData = "";
            currentProgData = "";
            lastTime = currTime;
        }
        
    char buf[256];
    sprintf(buf, "%g\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f\t %6.3f",
            currTime, data.mag[0], data.mag[1], data.mag[2], data.accel[0], data.accel[1], data.accel[2], 
            data.gyro[0], data.gyro[1], data.gyro[2], data.temp, data.pressure, currentAlt);
    //mFlightLog << buf;
    currentFlightData = currentFlightData + buf + '\n';
}

// write string to program data file and to console
void Log::write(std::string outputString){
    std::cout << outputString << std::endl;
    //mProgLog << outputString << '\n';
    currentProgData = currentProgData + outputString + '\n';
}

// write string to program data file with timestamp and to console
void Log::writeTime(std::string outputString){
    std::cout << outputString << std::endl;
    //mProgLog << outputString + " (" + std::to_string(sampleNumber) + " | " + std::to_string(elapsedTime()) << ")\n";
    //mProgLog << elapsedTime() << '\n';
    currentProgData = currentProgData + outputString + " (" + std::to_string(sampleNumber) + " | " + std::to_string(elapsedTime()) + ")\n";
}

// write string to program data file with special delimiters and timestamp for MATLAB postprocessing, and to console
void Log::writeDelim(std::string outputString){
    std::cout << outputString << std::endl;
    //mProgLog <<delim + outputString + " (" + std::to_string(sampleNumber) + " | " + std::to_string(elapsedTime()) + ")" + delim << '\n';
    currentProgData = currentProgData + delim + outputString + " (" + std::to_string(sampleNumber) + " | " + std::to_string(elapsedTime()) + ")" + delim + '\n';
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
