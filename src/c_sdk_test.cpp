//
// Created by Nathan Kau on 10/26/19.
//

#include "odrive_c_sdk.h"
#include <iostream>

int main(int argc, const char *argv[]) {
    const char *odrive_serial_number = "35722173822280";
    ODrive_t odrive_ptr = ODrive(odrive_serial_number);
    int result = initODrive(odrive_ptr);
    std::cerr << "Result: " << result << std::endl;

    runCalibration(odrive_ptr);
    do {
        std::cout << '\n' << "Press any key when calibration is done...";
    } while (std::cin.get() != '\n');
    allIdle(odrive_ptr);
    result = allReady(odrive_ptr);
    std::cerr << "Result: " << result << std::endl;
    setCurrentCtrlMode(odrive_ptr);
    float pos0, vel0, pos1, vel1;
    float cmd0 = 0.0;
    float cmd1 = 0.0;

    // seg faults!!
    // void* thread = controlODrive(odrive_ptr, cmd0, cmd1, &pos0, &vel0, &pos1, &vel1);
    // joinThread(thread);

    // Test 1
    controlODriveHelper(odrive_ptr, cmd0, cmd1, &pos0, &vel0, &pos1, &vel1);

    // Test 2
    // void* thread = controlODrive2(odrive_ptr, cmd0, cmd1, &pos0, &vel0, &pos1, &vel1);
    // joinThread(thread);

    return 1;
}
