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

    ThreadPool_t tp_ptr = ThreadPool();

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

    addODriveToThreadPool(tp_ptr, odrive_ptr);

    std::cout << "Starting timing tests..." << std::endl;
    int NUM_LOOPS = 10000;

    float total_sum = 0;
    float mid_sum = 0;
    for (int i = 0; i < NUM_LOOPS; i++) {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        controlODrive(tp_ptr, odrive_ptr, cmd0, cmd1, &pos0, &vel0, &pos1, &vel1);

        std::chrono::steady_clock::time_point mid = std::chrono::steady_clock::now();

        waitForThreads(tp_ptr);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        total_sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        mid_sum += std::chrono::duration_cast<std::chrono::microseconds>(mid - start).count();
    }
    float total_avg = total_sum / NUM_LOOPS;
    float mid_avg = mid_sum / NUM_LOOPS;
    std::cout << "Average time for command = " << total_avg << std::endl;
    std::cout << "Average time waiting for response = " << mid_avg << std::endl;
    
    return 1;
}
