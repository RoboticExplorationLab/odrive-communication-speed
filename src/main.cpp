#include <iostream>
#include "odrive_cpp_sdk.h"
#include <math.h>
#include <unistd.h>
#include <ctime>
#include <chrono>
#include <thread>

void communicateODrive0_thread(){
    std::string bl_sn = "60903547810103";
    std::string odrive_serial_numbers[1] = {bl_sn};
    std::string odrive_serial_numbers_map[2] = {bl_sn, bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 2;
    uint8_t num_odrvs = 1;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            num_odrvs,
            odrive_serial_numbers_map,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int16_t zeroeth_radian_in_encoder_ticks_[1] = {0};

    odrives.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        std::cout << "Setting to closed loop" << std::endl;
        result = odrives.allReady();
        std::cout << "Setting to current control" << std::endl;
        result = odrives.setCurrentCtrlMode();

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();

        int NUM_LOOPS = 10000;

        float sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.useTestFunction(10);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        float avg = sum / NUM_LOOPS;
        std::cout << "ODrive 0 test function time = " << avg << std::endl;


        sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            float odrives_encoders[num_motors] = {0.0f, 0.0f};
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.readEncoders(odrives_encoders);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        avg = sum / NUM_LOOPS;
        std::cout << "ODrive 0 encoder command time = " << avg << std::endl;

        sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            float odrive_current_des[4] = {0.0, 0.0, 0.0, 0.0};
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.setCurrentSetpoint(odrive_current_des);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        avg = sum / NUM_LOOPS;
        std::cout << "ODrive 0 current command time = " << avg << std::endl;

        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 0 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }

}

void communicateODrive1_thread(){
    std::string bl_sn = "35735059313992";
    std::string odrive_serial_numbers[1] = {bl_sn};
    std::string odrive_serial_numbers_map[2] = {bl_sn, bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 2;
    uint8_t num_odrvs = 1;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            num_odrvs,
            odrive_serial_numbers_map,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int16_t zeroeth_radian_in_encoder_ticks_[1] = {0};

    odrives.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        std::cout << "Setting to closed loop" << std::endl;
        result = odrives.allReady();
        std::cout << "Setting to current control" << std::endl;
        result = odrives.setCurrentCtrlMode();

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();

        int NUM_LOOPS = 10000;

        float sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.useTestFunction(10);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        float avg = sum / NUM_LOOPS;
        std::cout << "ODrive 1 test function time = " << avg << std::endl;


        sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            float odrives_encoders[num_motors] = {0.0f, 0.0f};
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.readEncoders(odrives_encoders);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        avg = sum / NUM_LOOPS;
        std::cout << "ODrive 1 encoder command time = " << avg << std::endl;

        sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            float odrive_current_des[4] = {0.0, 0.0, 0.0, 0.0};
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.setCurrentSetpoint(odrive_current_des);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        avg = sum / NUM_LOOPS;
        std::cout << "ODrive 1 current command time = " << avg << std::endl;

        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 1 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }
}


void communicateODrive2_thread(){
    std::string bl_sn = "35765125264712";
    std::string odrive_serial_numbers[1] = {bl_sn};
    std::string odrive_serial_numbers_map[2] = {bl_sn, bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 2;
    uint8_t num_odrvs = 1;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            num_odrvs,
            odrive_serial_numbers_map,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int16_t zeroeth_radian_in_encoder_ticks_[1] = {0};

    odrives.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        std::cout << "Running calibration" << std::endl;
        result = odrives.runCalibration();

        std::cout << "Setting to closed loop" << std::endl;
        result = odrives.allReady();
        std::cout << "Setting to current control" << std::endl;
        result = odrives.setCurrentCtrlMode();

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();

        int NUM_LOOPS = 10000;

        float sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.useTestFunction(10);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        float avg = sum / NUM_LOOPS;
        std::cout << "ODrive 2 test function time = " << avg << std::endl;


        sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            float odrives_encoders[num_motors] = {0.0f, 0.0f};
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.readEncoders(odrives_encoders);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        avg = sum / NUM_LOOPS;
        std::cout << "ODrive 2 encoder command time = " << avg << std::endl;

        sum = 0;
        for (int i = 0; i < NUM_LOOPS; i++) {
            float odrive_current_des[4] = {0.0, 0.0, 0.0, 0.0};
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            odrives.setCurrentSetpoint(odrive_current_des);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        avg = sum / NUM_LOOPS;
        std::cout << "ODrive 2 current command time = " << avg << std::endl;

        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 2 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }
}

void encoderFuncODrive2_thread(){
    std::string bl_sn = "35765125264712";
    std::string odrive_serial_numbers[1] = {bl_sn};
    std::string odrive_serial_numbers_map[2] = {bl_sn, bl_sn};

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};

    uint8_t num_motors = 2;
    uint8_t num_odrvs = 1;

    odrive::CppSdk odrives(
            odrive_serial_numbers,
            num_odrvs,
            odrive_serial_numbers_map,
            odrive_position_per_motor,
            odrive_encoder_ticks_per_radian_per_motor,
            motor_relative_to_prior_motor,
            num_motors
    );

    int16_t zeroeth_radian_in_encoder_ticks_[1] = {0};

    odrives.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

    int result = odrives.init();

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    }
    else {
        std::cout << "Running calibration" << std::endl;
        result = odrives.runCalibration();

        std::cout << "Setting to closed loop" << std::endl;
        result = odrives.allReady();
        std::cout << "Setting to current control" << std::endl;
        result = odrives.setCurrentCtrlMode();

        std::chrono::steady_clock::time_point true_start = std::chrono::steady_clock::now();

        float encoder = odrives.getEncodersFunction(0.0);

        std::cout << "Encoder value = " << encoder << std::endl;

        std::chrono::steady_clock::time_point true_end = std::chrono::steady_clock::now();

        std::cout << "ODrive 2 total time = "
                  << std::chrono::duration_cast<std::chrono::microseconds>(true_end - true_start).count() << std::endl;
    }
}

int main(int argc, const char * argv[]) {

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

//    std::thread odrv0(communicateODrive0_thread);
//    std::thread odrv1(communicateODrive1_thread);
//    std::thread odrv2(communicateODrive2_thread);

//    odrv0.join();
//    odrv1.join();
//    odrv2.join();

    std::thread encoder(encoderFuncODrive2_thread);

    encoder.join();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Main thread time = " <<std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;

//    std::thread thread1(thread_func1);
//    std::thread thread2(thread_func2);
//
//    thread1.join();
//    thread2.join();

    return 1;
}
