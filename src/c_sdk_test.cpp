//
// Created by Nathan Kau on 10/26/19.
//

#include "odrive_c_sdk.h"
#include <iostream>
#include <chrono>
#include <unistd.h>

int main(int argc, const char **argv) {
    /* Tests that the c wrapper for the odrive sdk works.
     * For use on the raspberry pi, run 'sudo ./c_sdk_test [arguments]'
     * Pass no arguments if you want to skip the calibration procedure, 
     * otherwise pass c, as in 'sudo ./c_sdk_test c' to 
     * do the calibration procedure before running the test
     */

    ThreadPool_t tp_ptr = ThreadPool();

    const char *odrv0_serial_number = "35722173822280";
    const char *odrv1_serial_number = "60894957285687";
    ODrive_t odrv0 = ODrive(odrv0_serial_number);
    std::cout << "Connected odrive0" << std::endl;
    ODrive_t odrv1 = ODrive(odrv1_serial_number);
    std::cout << "Connected odrive1" << std::endl;
    
    usleep(5000000);

    int result = initODrive(odrv0);
    std::cout << "Result from initializing odrv0: " << result << std::endl;    
    
    usleep(5000000);

    result = initODrive(odrv1);
    std::cout << "Result from initializing odrv1: " << result << std::endl;

    if(result != 0) {
        std::cerr << "ERROR: Could not initialize ODrives" << std::endl;
        return 1;
    }

    if(argc > 1 && argv[1][0] == 'c') {
        runCalibration(odrv0);
        runCalibration(odrv1);
        do {
            std::cout << '\n' << "Press any key when calibration is done...";
        } while (std::cin.get() != '\n');
    }

    allIdle(odrv0);
    allIdle(odrv1);
    result = allReady(odrv0) & allReady(odrv1);
    std::cout << "Result: " << result << std::endl;
    setCurrentCtrlMode(odrv0);
    setCurrentCtrlMode(odrv1);
    float pos0_0, vel0_0, pos1_0, vel1_0;
    float pos0_1, vel0_1, pos1_1, vel1_1;
    float cmd0 = 0.0;
    float cmd1 = 0.0;

    addODriveToThreadPool(tp_ptr, odrv0);
    addODriveToThreadPool(tp_ptr, odrv1);

    std::cout << "Starting timing tests..." << std::endl;
    int NUM_LOOPS = 10000;

    float total_sum = 0;
    float mid_sum = 0;
    for (int i = 0; i < NUM_LOOPS; i++) {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        controlODrive(tp_ptr, odrv0, cmd0, cmd1, &pos0_0, &vel0_0, &pos1_0, &vel1_0);
        controlODrive(tp_ptr, odrv1, cmd0, cmd1, &pos0_1, &vel0_1, &pos1_1, &vel1_1);

        std::chrono::steady_clock::time_point mid = std::chrono::steady_clock::now();

        waitForThreads(tp_ptr);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        total_sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        mid_sum += std::chrono::duration_cast<std::chrono::microseconds>(mid - start).count();
    }
    float total_avg = total_sum / NUM_LOOPS;
    float mid_avg = mid_sum / NUM_LOOPS;
    std::cout << "Average time for command = " << total_avg << std::endl;
    std::cout << "Average time sending commands = " << mid_avg << std::endl;
    
    return 1;
}
