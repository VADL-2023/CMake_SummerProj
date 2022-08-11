#include "sensors.h"
#include "config_IMU.hpp" //Copy this file into MainDriver includes -> currently in IMU
#include "pigpio.h"
#include <ctime>
#include "ezasyncdata.h"
#include "Log2.h"

//#include <chrono>
//#include <iostream>
//#include <sys/time.h>
//#include <ctime>

int main(){
    //Initialize pigpio (for servos)
    gpioInitialise();
    
    // IMU Connection and Configuration
    VnSensor* mVN = new VnSensor();
    
    uint16_t nMeasurements = 3600;
    float targetPressure = 97;
    bool targetDetected = false;
    
    //Initialize Log object to save data
    Log2 mLog("VT Flight Data Log 2", "VT Program Data Log 2", mVN);
    
    mLog.write("Date: 8/11");
    mLog.write("Test Number: 2");
    mLog.write("Number of samples: ");
    mLog.write(to_string(nMeasurements));
    mLog.write("Expected run time (us): ");
    mLog.write(to_string(nMeasurements/.02));
    mLog.write("Deployment pressure: ");
    mLog.write(to_string(targetPressure));
    mLog.write("-----------------------------");
    
    auto mTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    mLog.write(to_string(mTime)); 
    mLog.write("IMU Connecting");
    mTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    mLog.write(to_string(mTime));
    
    mVN->connect(IMU_PORT,IMU_BAUD_RATE);
    if (!mVN->isConnected()){
        throw "IMU Failed to Connect";
    }else{
        mLog.write("IMU Connected");
        mTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        mLog.write(to_string(mTime));
    }

    ImuMeasurementsRegister response1;
    time_t t_start1, t_end1;
    //time_t t_start2, t_end2;
    //Clock 100 measurements(using simplest Library method)
    time(&t_start1);

    auto mTimeStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    for (int i = 0; i < nMeasurements; ++i){
        response1 = mVN->readImuMeasurements();
        
        if (response1.pressure < targetPressure && !targetDetected){
            targetDetected = true;
            mLog.write("Actuate Servos!");
            mLog.write(to_string(response1.pressure));
            mTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            mLog.write(to_string(mTime));
         }
         
         if (response1.pressure > (targetPressure + 1) && targetDetected){
            targetDetected = false;
            mLog.write("Descending back down!");
            mLog.write(to_string(response1.pressure));
            mTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            mLog.write(to_string(mTime));
         }
        auto mTimeEnd = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        double elapsedTime = double(mTimeEnd - mTimeStart);
        mLog.write(response1, elapsedTime);     
    }
    time(&t_end1);
    
    //Log Results
    double elapsed_1 = double(t_end1 - t_start1);
    
    mLog.write("Time taken (s): "); 
    mLog.write(to_string(elapsed_1));
    mLog.write("Frequency: ");
    mLog.write(to_string(nMeasurements/elapsed_1));
    
    if (IMU_ACTIVE){
    mLog.write("IMU: Disconnecting");

    mVN->disconnect();

    mLog.write("IMU: Disconnected");
    mTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    mLog.write(to_string(mTime));
    
    }
    
    delete mVN;
    mLog.write("END");
	return 0;
}
