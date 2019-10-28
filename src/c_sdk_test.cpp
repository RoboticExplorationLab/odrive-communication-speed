//
// Created by Nathan Kau on 10/26/19.
//

#include "odrive_c_sdk.h"
#include <iostream>

int main(int argc, const char **argv) {
    /* Tests that the c wrapper for the odrive sdk works.
     * For use on the raspberry pi, run 'sudo ./c_sdk_test [arguments]'
     * Pass no arguments if you want to skip the calibration procedure, 
     * otherwise pass c, as in 'sudo ./c_sdk_test c' to 
     * do the calibration procedure before running the test
     */

    const char *odrive_serial_number = "35722173822280";
    ODrive_t odrive_ptr = ODrive(odrive_serial_number);
    int result = initODrive(odrive_ptr);
    std::cout << "Result: " << result << std::endl;

    if(result != 0) {
        std::cerr << "ERROR: No ODrives available" << std::endl;
        return 1;
    }

    if(argc > 1 && argv[1][0] == 'c') {
        runCalibration(odrive_ptr);
        do {
            std::cout << '\n' << "Press any key when calibration is done...";
        } while (std::cin.get() != '\n');
    }

    allIdle(odrive_ptr);
    result = allReady(odrive_ptr);
    std::cout << "Result: " << result << std::endl;
    setCurrentCtrlMode(odrive_ptr);
    float pos0, vel0, pos1, vel1;
    float cmd0 = 0.0;
    float cmd1 = 0.0;

    // Test 1
    controlODriveHelper(odrive_ptr, cmd0, cmd1, &pos0, &vel0, &pos1, &vel1);
    std::cout << pos0 << std::endl;
    
    // Test 2    
    void* thread = controlODrive(odrive_ptr, cmd0, cmd1, &pos0, &vel0, &pos1, &vel1);
    joinThread(thread);

    return 1;
}
