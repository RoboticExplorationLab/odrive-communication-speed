//
// Created by Aaron Schultz on 1/25/20.
//

#include "odrive_c_sdk.h"
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <signal.h>

volatile sig_atomic_t runLoop = 1;

void intHandle(int signum) {
    runLoop = 0;
}

int main(int argc, const char **argv) {
    /* Tests that the c wrapper for the odrive sdk works.
     * For use on the raspberry pi, run 'sudo ./c_sdk_test [arguments]'
     * Pass no arguments if you want to skip the calibration procedure, 
     * otherwise pass c, as in 'sudo ./c_sdk_test c' to 
     * do the calibration procedure before running the test
     */

    ThreadPool_t tp_ptr = ThreadPool();

    std::vector<const char*> serial_numbers = {"35722173822280"};
    std::vector<ODrive_t> odrives;

    for (size_t i = 0; i < serial_numbers.size(); i++) {
        odrives.push_back(ODrive(serial_numbers[i]));
    }

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

        do {
            std::cout << '\n' << "Press any key when calibration is done...";
        } while (std::cin.get() != '\n');
    }

    for (ODrive_t odrv: odrives) {
        allIdle(odrv);
        setCurrentCtrlMode(odrv);
        addODriveToThreadPool(tp_ptr, odrv);
    }

    float cmd0 = 0.0;
    float cmd1 = 0.0;

    struct meas_t {
        float pos0;
        float pos1;
        float vel0;
        float vel1;
    };

    std::vector<meas_t> measurements(odrives.size(), {0.0, 0.0, 0.0, 0.0});

    signal(SIGINT, intHandle);

    std::cout << "Starting pid test..." << std::endl;

    for (ODrive_t odrv: odrives) {
        setWatchdogTimeout(odrv, 0.01);
    }

    float current = 0.0;
    while(runLoop) {

        size_t NUM_LOOPS = 10000;

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        for (size_t l = 0; l < NUM_LOOPS; l++) {
            for (size_t i = 0; i < odrives.size(); i++) {
                ODrive_t odrv = odrives[i];
                meas_t meas = measurements[i];
                current = 0.004*(0 - meas.pos0) + 0.0001 * (0 - meas.vel0);
                if (current > 2) current = 2.0;
                if (current < -2) current = -2.0;
                controlODrive(tp_ptr, odrv, current, cmd1, &meas.pos0, &meas.vel0, &meas.pos1, &meas.vel1);
            }

            waitForThreads(tp_ptr);
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        float total_sum = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        float total_avg = total_sum / NUM_LOOPS;
        std::cout << "Average time for control loop = " << total_avg << std::endl;

    }
    std::cout << "Ending control loop" << std::endl;

    for (ODrive_t odrv : odrives) {
        setCurrents(odrv, 0.0, 0.0);
    }

    for (ODrive_t odrv: odrives) {
        setWatchdogTimeout(odrv, 0.0);
    }

    return 1;
}
