#ifndef ODRIVE_C_SDK_H
#define ODRIVE_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void* ODrive_t;

void* ODrive(const char* serial_number);
int initODrive(ODrive_t odrive);

#ifdef __cplusplus
}
#endif

#endif 