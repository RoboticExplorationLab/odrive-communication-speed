#include "odrive_c_sdk.h"
#include "odrive_cpp_sdk.h"
#include "odrive_threadpool.h"

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


ThreadPool_t ThreadPool() {
    return new ODriveThreadPool();
}


void addODriveToThreadPool(ThreadPool_t tp, ODrive_t odrive) {
    auto typed_ptr = static_cast<ODriveThreadPool *>(tp);
    typed_ptr->addOdriveThread(odrive);
}


void destroyThreadPool(ThreadPool_t tp) {
    delete static_cast<ODriveThreadPool *>(tp);
}


void destroyODrive(ODrive_t odrive) {
    delete static_cast<odrive::CppSdk *>(odrive);
}


int initODrive(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->init();
}


int runCalibration(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->runCalibration();
}


int allReady(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->allReady();
}


int setCurrentCtrlMode(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->setCurrentCtrlMode();
}


int allIdle(ODrive_t odrive) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    return typed_ptr->allIdle();
}

int setCurrents(ODrive_t odrive, float cmd0, float cmd1) {
    auto typed_ptr = static_cast<odrive::CppSdk *>(odrive);
    const float currents[2] = {cmd0, cmd1};
    return typed_ptr->setCurrentSetpoint(currents);
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


void controlODrive(ThreadPool_t tp, ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1) {
    auto typed_ptr = static_cast<ODriveThreadPool *>(tp);
    typed_ptr->schedule(odrive, [=]{controlODriveHelper(odrive, cmd0, cmd1, pos0, vel0, pos1, vel1); });
}


void waitForThreads(ThreadPool_t tp) {
    auto typed_ptr = static_cast<ODriveThreadPool *>(tp);
    typed_ptr->wait();
}


#ifdef __cplusplus
}
#endif
