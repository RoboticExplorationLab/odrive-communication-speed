//
// Created by Nathan Kau on 10/26/19.
//

#include "odrive_c_sdk.h"

int main(int argc, const char * argv[]) {
    const char *odrive_serial_number = "35722173822280";
    ODrive_t odrive_ptr = ODrive(odrive_serial_number);
    int result = initODrive(odrive_ptr);
    std::cerr << "Result: " << result << std::endl;

//    runCalibration(odrive_ptr);

    return 1;
}