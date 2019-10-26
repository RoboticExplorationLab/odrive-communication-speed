#include "odrive_c_sdk.h"
#include "odrive_cpp_sdk.h"
#include <thread>

#ifdef __cplusplus
extern "C" {
#endif

ODrive_t ODrive(const char *serial_number) {

    std::string serial_num(serial_number);

    bool odrive_position_per_motor[2] = {false, true};
    bool motor_relative_to_prior_motor[1] = {false};
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};
    uint8_t num_motors = 2;

    return new odrive::CppSdk(serial_num, odrive_position_per_motor, odrive_encoder_ticks_per_radian_per_motor,
                              motor_relative_to_prior_motor, num_motors);
}


void destroyODrive(ODrive_t odrive) {
    delete[] static_cast<odrive::CppSdk *>(odrive);
}


int initODrive(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->init();
}


void runCalibration(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    typed_ptr->runCalibration();
}


void allReady(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    typed_ptr->allReady();
}


void setCurrentCtrlMode(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    typed_ptr->setCurrentCtrlMode();
}


int allIdle(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->allIdle();
}


void sendCurrentGetEncoder(ODrive_t odrive, const odrive::current_command_t &current,
                           odrive::encoder_measurements_t &encoder) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    typed_ptr->getEncodersStructFunction(current, &encoder);
}


void *controlODrive(ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1) {
    odrive::current_command_t current_cmd = {cmd0, cmd1};
    odrive::encoder_measurements_t encoder_meas;
    return new std::thread(sendCurrentGetEncoder, std::ref(odrive), std::ref(current_cmd), std::ref(encoder_meas));
}


void joinThread(void *thread) {
    auto typed_ptr = static_cast<std::thread *>(thread);
    typed_ptr->join();
}


#ifdef __cplusplus
}
#endif