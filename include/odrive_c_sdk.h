#ifndef ODRIVE_C_SDK_H
#define ODRIVE_C_SDK_H

#include "odrive_cpp_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif
    
typedef void *ODrive_t;
void *ODrive(const char *serial_number);
int initODrive(ODrive_t odrive);
void destroyODrive(ODrive_t odrive);
int initODrive(ODrive_t odrive);
void runCalibration(ODrive_t odrive);
int allReady(ODrive_t odrive);
void setCurrentCtrlMode(ODrive_t odrive);
int allIdle(ODrive_t odrive);
void *controlODrive(ODrive_t
                    odrive, float cmd0,
                    float cmd1,
                    float *pos0,
                    float *vel0,
                    float *pos1,
                    float *vel1);
void joinThread(void *thread);

#ifdef __cplusplus
}
#endif

void sendCurrentGetEncoder(ODrive_t odrive, const odrive::current_command_t &current,
                           odrive::encoder_measurements_t &encoder);

#endif 
