//
// Created by Nathan Kau on 10/26/19.
//

#include "odrive_c_sdk.h"
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>

int main(int argc, const char **argv) {
    /* Tests that the c wrapper for the odrive sdk works.
     * For use on the raspberry pi, run 'sudo ./c_sdk_test [arguments]'
     * Pass no arguments if you want to skip the calibration procedure, 
     * otherwise pass c, as in 'sudo ./c_sdk_test c' to 
     * do the calibration procedure before running the test
     */

    ThreadPool_t tp_ptr = ThreadPool();

    std::vector<const char*> serial_numbers = {"35619029856331", "35799416713291", "59752448340023", "35726434842440", "35799416778827", "35795121942603"};
    std::vector<ODrive_t> odrives;

    for (size_t i = 0; i < serial_numbers.size(); i++) {
        odrives.push_back(ODrive(serial_numbers[i]));
    }

    // const char *odrv0_serial_number = "59752448340023"; //"35722173822280";
    // const char *odrv1_serial_number = "35722173822280"; //"60894957285687";
    // const char *odrv2_serial_number = "35619062370891";
    // ODrive_t odrv0 = ODrive(odrv0_serial_number);
    // std::cout << "Connected odrive0" << std::endl;
    // ODrive_t odrv1 = ODrive(odrv1_serial_number);
    // std::cout << "Connected odrive1" << std::endl;
    // ODrive_t odrv2 = ODrive(odrv2_serial_number);
    // std::cout << "Connected odrive2" << std::endl;
    

    // int result = initODrive(odrv0);
    // std::cout << "Result from initializing odrv0: " << result << std::endl;    
    

    // result = initODrive(odrv1);
    // std::cout << "Result from initializing odrv1: " << result << std::endl;

    // usleep(1000000);

    // int result2 = initODrive(odrv2);
    // std::cout << "Result from initializing odrv2: " << result2 << std::endl;
    for (size_t i = 0; i < odrives.size(); i++) {
        if(initODrive(odrives[i])) {
            std::cerr << "ERROR: Could not initialize ODrive" << i << std::endl;
            return 1;
        }
    }

    if(argc > 1 && argv[1][0] == 'c') {
        for (ODrive_t odrv: odrives) {
            runCalibration(odrv);
        }
        // runCalibration(odrv0);
        // runCalibration(odrv1);
        //runCalibration(odrv2);
        do {
            std::cout << '\n' << "Press any key when calibration is done...";
        } while (std::cin.get() != '\n');
    }

    for (ODrive_t odrv: odrives) {
        allIdle(odrv);
        setCurrentCtrlMode(odrv);
        addODriveToThreadPool(tp_ptr, odrv);
    }
    // allIdle(odrv0);
    // allIdle(odrv1);
    // allIdle(odrv2);
    // result = allReady(odrv0) | allReady(odrv1); // | allReady(odrv2);
    // std::cout << "Odrives ready: " << result << std::endl;
    // setCurrentCtrlMode(odrv0);
    // setCurrentCtrlMode(odrv1);
    // setCurrentCtrlMode(odrv2);
    // float pos0_0, vel0_0, pos1_0, vel1_0;
    // float pos0_1, vel0_1, pos1_1, vel1_1;
    // float pos0_2, vel0_2, pos1_2, vel1_2;
    float cmd0 = 0.0;
    float cmd1 = 0.0;

    struct meas_t {
        float pos0;
        float pos1;
        float vel0;
        float vel1;
    };

    std::vector<meas_t> measurements(odrives.size(), {0.0, 0.0, 0.0, 0.0});

    // addODriveToThreadPool(tp_ptr, odrv2);

    std::cout << "Starting pid test..." << std::endl;

    float current = 0.0;
    while(true) {

        int NUM_LOOPS = 10000;

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        for (int i = 0; i < NUM_LOOPS; i++) {
            for (size_t i = 0; i < odrives.size(); i++) {
                ODrive_t odrv = odrives[i];
                meas_t meas = measurements[i];
                current = 0.004*(0 - meas.pos0) + 0.0001 * (0 - meas.vel0);
                if (current > 2) current = 2.0;
                if (current < -2) current = -2.0;
                controlODrive(tp_ptr, odrv, current, cmd1, &meas.pos0, &meas.vel0, &meas.pos1, &meas.vel1);
            }
            // controlODrive(tp_ptr, odrv0, current, cmd1, &pos0_0, &vel0_0, &pos1_0, &vel1_0);
            // controlODrive(tp_ptr, odrv1, 0.0, 0.0, &pos0_1, &vel0_1, &pos1_1, &vel1_1);
            waitForThreads(tp_ptr);
            //std::cout << pos0_0 << ", " << vel0_0 << std::endl;
            // current = 0.004*(pos0_1 - pos0_0) - 0.0001*vel0_0;
            // if (current > 5) current = 5.0;
            // if (current < -5) current = -5.0;
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        float total_sum = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        float total_avg = total_sum / NUM_LOOPS;
        std::cout << "Average time for control loop = " << total_avg << std::endl;

    }

    return 1;
}
