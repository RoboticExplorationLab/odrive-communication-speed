#include "odrive_c_sdk.h"
#include "odrive_cpp_sdk.h"
#include <thread>

#ifdef __cplusplus
extern "C" {
#endif

ODrive_t ODrive(const char *serial_number) {
    std::string serial_num(serial_number);
    bool motor_position_map[2] = {0, 1};
    float odrive_encoder_ticks_per_radian_per_motor[1] = {954.949};
    uint8_t num_motors = 2;

    return new odrive::CppSdk(serial_num, motor_position_map, odrive_encoder_ticks_per_radian_per_motor,
                              num_motors);
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


int allReady(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->allReady();
}


void setCurrentCtrlMode(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    typed_ptr->setCurrentCtrlMode();
}


int allIdle(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->allIdle();
}


void controlODriveHelper(ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1) {
    odrive::current_command_t current_cmd = {cmd0, cmd1};
    odrive::encoder_measurements_t encoder_meas;

    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    typed_ptr->getEncodersStructFunction(current_cmd, encoder_meas);

    *pos0 = encoder_meas.encoder_pos_axis0;
    *vel0 = encoder_meas.encoder_vel_axis0;
    *pos1 = encoder_meas.encoder_pos_axis1;
    *vel1 = encoder_meas.encoder_vel_axis1;
}


void *controlODrive(ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1) {
    return new std::thread(controlODriveHelper, std::ref(odrive), std::ref(cmd0), std::ref(cmd1), std::ref(pos0),
                           std::ref(vel0), std::ref(pos1), std::ref(vel1));
}


//
//void *controlODrive(ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1) {
//    odrive::current_command_t current_cmd = {cmd0, cmd1};
//    odrive::encoder_measurements_t encoder_meas;
//
//    // BUG: encoder_meas is passed by reference but once the function returns, encoder_meas is freed so we get a broken reference.
//    return new std::thread(sendCurrentGetEncoder, std::ref(odrive), std::ref(current_cmd), std::ref(encoder_meas));
//}


void joinThread(void *thread) {
    auto typed_ptr = static_cast<std::thread *>(thread);
    typed_ptr->join();
    delete [] typed_ptr;
}


#ifdef __cplusplus
}
#endif
