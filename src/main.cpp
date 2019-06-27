#include <iostream>
#include "odrive_cpp_sdk.h"
#include <math.h>
#include <unistd.h>
#include <ctime>
#include <chrono>
#include <thread>

void encoderFuncODrive0_thread(){
    std::string bl_sn = "60903547810103";
    std::string odrive_serial_numbers[1] = {bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 2;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        odrive::current_command_t cmd;
        cmd.current_axis0 = 0.5;
        cmd.current_axis1 = 0.5;

        odrive::encoder_measurements_t meas;

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();

        int NUM_LOOPS = 10000;

        float sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            result = odrives.getEncodersStructFunction(cmd, meas);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        float avg = sum / NUM_LOOPS;
        std::cout << "ODrive 0 test function time = " << avg << std::endl;
        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 0 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }
}

void encoderFuncODrive1_thread(){
    std::string bl_sn = "35735059313992";
    std::string odrive_serial_numbers[1] = {bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 2;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        odrive::current_command_t cmd;
        cmd.current_axis0 = 0.5;
        cmd.current_axis1 = 0.5;

        odrive::encoder_measurements_t meas;

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();
        int NUM_LOOPS = 10000;

        float sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            result = odrives.getEncodersStructFunction(cmd, meas);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        float avg = sum / NUM_LOOPS;
        std::cout << "ODrive 1 test function time = " << avg << std::endl;
        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 1 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }
}

void encoderFuncODrive2_thread(){
    std::string bl_sn = "35765125264712";
    std::string odrive_serial_numbers[1] = {bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 1;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        odrive::current_command_t cmd;
        cmd.current_axis0 = 0.0;
        cmd.current_axis1 = 0.0;

        odrive::encoder_measurements_t meas;

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();
        int NUM_LOOPS = 10000;

        float sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            result = odrives.getEncodersStructFunction(cmd, meas);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        float avg = sum / NUM_LOOPS;
        std::cout << "ODrive 2 test function time = " << avg << std::endl;
        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 2 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }
}

int main(int argc, const char * argv[]) {

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

//    std::thread encoder0(encoderFuncODrive0_thread);
//    std::thread encoder1(encoderFuncODrive1_thread);
    std::thread encoder2(encoderFuncODrive2_thread);

//    encoder0.join();
//    encoder1.join();
    encoder2.join();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Main thread time = " <<std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;

    return 1;
}
