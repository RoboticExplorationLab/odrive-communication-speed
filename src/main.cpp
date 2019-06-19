#include <iostream>
#include "odrive_cpp_sdk.h"
#include <math.h>
#include <unistd.h>
#include <ctime>
#include <chrono>

int main(int argc, const char * argv[]) {

	std::string bl_sn = "35765125264712"; //"60903547810103";
	std::string fl_sn = "35735059313992";

    std::string odrive_serial_numbers[2] = {bl_sn,fl_sn};
	std::string odrive_serial_numbers_map[4] = {bl_sn, bl_sn, fl_sn, fl_sn};
	int16_t zeroeth_radian_in_encoder_ticks_[2] = { -200, 0 };
	bool odrive_position_per_motor[4] = {false, true, false, true};
	bool motor_relative_to_prior_motor[2] = {false, false};
	// odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
	float odrive_encoder_ticks_per_radian_per_motor[2] = {954.949, 954.949};
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

	odrives.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

	int result = odrives.init();

	if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
	    std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
	    return EXIT_FAILURE;
	}

//	std::cout << "Running calibration" << std::endl;
//	//result = odrives.runCalibration();
//	std::cout << "Setting to closed loop" << std::endl;
	result = odrives.allReady();
//	std::cout << "Setting to current control" << std::endl;
	result = odrives.setCurrentCtrlMode();
//
//	uint8_t odrive_motor_current_errors[num_motors] = {0}; //change 4 to num_motors on linux
//	result = odrives.checkErrors(odrive_motor_current_errors);
//	std::cout << "odrive.checkErrors got result: " << result << " and values: [" << std::to_string(odrive_motor_current_errors[0]) << "," << std::to_string(odrive_motor_current_errors[1]) << "," << std::to_string(odrive_motor_current_errors[2]) << "," << std::to_string(odrive_motor_current_errors[3]) << "]" << std::endl;
//
//	float odrives_encoders[num_motors] = {0}; //change 4 to num_motors on linux
//	result = odrives.readEncoders(odrives_encoders);
//	std::cout << "Encoders: [" << std::to_string(odrives_encoders[0]) << "," << std::to_string(odrives_encoders[1]) << "," << std::to_string(odrives_encoders[2]) << "," << std::to_string(odrives_encoders[3]) << "]" << std::endl;
//
//	float odrive_current_des[4] = {0.0, 0.0, 0.6, 0.0};
//	result = odrives.setCurrentSetpoint(odrive_current_des);
//	std::cout << "Setting current with result: " << result << std::endl;

    int NUM_LOOPS = 1000;
	float sum = 0;
	for (int i = 0; i < NUM_LOOPS; i++){
        float odrives_encoders[num_motors] = {0};
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        odrives.readEncoders(odrives_encoders);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	}
	float avg = sum / NUM_LOOPS;
	std::cout << avg << std::endl;

    sum = 0;
	for (int i = 0; i < NUM_LOOPS; i++){
        float odrive_current_des[4] = {0.0, 0.0};
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        odrives.setCurrentSetpoint(odrive_current_des);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        sum += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
    avg = sum / NUM_LOOPS;
    std::cout << avg << std::endl;


}
