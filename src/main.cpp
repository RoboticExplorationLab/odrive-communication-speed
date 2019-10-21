#include <iostream>
#include "odrive_cpp_sdk.h"
#include <math.h>
#include <unistd.h>
#include <ctime>
#include <chrono>
#include <thread>

void sendCurrentGetEncoder(odrive::CppSdk& odrive, const odrive::current_command_t& current, odrive::encoder_measurements_t& encoder){
    odrive.getEncodersStructFunction(current, &encoder);
}

int main(int argc, const char * argv[]) {
    int result;

    std::string bl_sn = "60903547810103";
    std::string br_sn = "35735059313992";

    // ODrive in the tupperware with test hardware
    std::string test_sn = "35722173822280";

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};


    // initialize back left ODrive
    uint8_t num_motors = 2;

    odrive::CppSdk odrive_bl(
            bl_sn,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    // initialize back right ODrive
    odrive::CppSdk odrive_br(
            br_sn,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    // initialize test ODrive
    odrive::CppSdk odrive_test(
            test_sn,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    result = odrive_test.init();
    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cerr << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
        return result;
    }

    result = odrive_bl.init();
    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cerr << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
        return result;
    }

    result = odrive_br.init();
    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cerr << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
        return result;
    }

    odrive_br.runCalibration();
    odrive_bl.runCalibration();
    odrive_test.runCalibration();

    do
    {
        std::cout << '\n' << "Press any key when calibration is done...";
    } while (std::cin.get() != '\n');

    odrive_br.allReady();
    odrive_bl.allReady();
    odrive_test.allReady();

    odrive_br.setCurrentCtrlMode();
    odrive_bl.setCurrentCtrlMode();
    odrive_test.setCurrentCtrlMode();

    odrive::current_command_t current_cmd_test = {0.0, 0.0};
    odrive::current_command_t current_cmd_b = {0.0, 0.0};
    odrive::encoder_measurements_t encoder_meas_test;
    odrive::encoder_measurements_t encoder_meas_bl;
    odrive::encoder_measurements_t encoder_meas_br;

    int NUM_LOOPS = 10000;

    float total_sum = 0;
    float mid_sum = 0;
    for (int i = 0; i < NUM_LOOPS; i++) {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        std::thread test(sendCurrentGetEncoder, std::ref(odrive_test), std::ref(current_cmd_test), std::ref(encoder_meas_test));
        std::thread bl(sendCurrentGetEncoder, std::ref(odrive_bl), std::ref(current_cmd_b), std::ref(encoder_meas_bl));
        std::thread br(sendCurrentGetEncoder, std::ref(odrive_br), std::ref(current_cmd_b), std::ref(encoder_meas_br));
        std::chrono::steady_clock::time_point mid = std::chrono::steady_clock::now();

        test.join();
        bl.join();
        br.join();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        total_sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        mid_sum += std::chrono::duration_cast<std::chrono::microseconds>(mid - start).count();
    }
    float total_avg = total_sum / NUM_LOOPS;
    float mid_avg = mid_sum / NUM_LOOPS;
    std::cout << "ODrive total function time = " << total_avg << std::endl;
    std::cout << "ODrive total function time = " << mid_avg << std::endl;

    return 1;
}
