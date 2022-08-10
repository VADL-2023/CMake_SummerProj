#include "../include/Log2.h"

Log2::Log2(std::ofstream &flight, std::ofstream &program):
mFlightLog(flight), mProgLog(program)
{
    if (!(mFlightLog.is_open() && mProgLog.is_open())){
        std::cout << "Error opening files";
    }else{
        mFlightLog << "Mag X, Mag Y, Mag Z, Accel X, Accel Y, Accel Z, Y, P, R, Temp (C), Pressure (kPa)\n";
        mProgLog << "Successfully opened both file output streams \n";
    }

}

// May have to add 'vn::sensors::' before ImuMeasurementsRegister, hopefully not
void Log2::write(ImuMeasurementsRegister& data){
    char* buf[256];
    sprintf(buf, "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n", data.mag[0], data.mag[1], data.mag[2], data.accel[0], data.accel[1], data.accel[2], data.gyro[0], data.gyro[1], data.gyro[2]);
    mFlightLog << buf;
}

void Log2::write(std::string& outputString){
    mProgLog << outputString << '\n';
}