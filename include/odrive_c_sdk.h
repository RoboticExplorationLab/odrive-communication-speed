#ifndef ODRIVE_C_SDK_H
#define ODRIVE_C_SDK_H

#include "odrive_cpp_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *ODrive_t;
typedef void *ThreadPool_t;
void *ODrive(const char *serial_number);
void *ThreadPool();
void addODriveToThreadPool(ThreadPool_t tp, ODrive_t odrive);
void destroyThreadPool(ThreadPool_t tp);
int initODrive(ODrive_t odrive);
void destroyODrive(ODrive_t odrive);
int initODrive(ODrive_t odrive);
int runCalibration(ODrive_t odrive);
int allReady(ODrive_t odrive);
void setCurrentCtrlMode(ODrive_t odrive);
int allIdle(ODrive_t odrive);
void controlODriveHelper(ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1);
void controlODrive(ThreadPool_t tp, ODrive_t odrive, float cmd0, float cmd1, float *pos0, float *vel0, float *pos1, float *vel1);
void waitForThreads(ThreadPool_t tp);

#ifdef __cplusplus
}
#endif

void sendCurrentGetEncoder(ODrive_t odrive, const odrive::current_command_t &current,
                           odrive::encoder_measurements_t &encoder);

#endif 
