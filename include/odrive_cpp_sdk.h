#ifndef ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_
#define ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_


#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
//#include "endian.h"

typedef std::vector<uint8_t> commBuffer;

#define ODRIVE_SDK_USB_VENDORID     4617 //decimal for 0x1209
#define ODRIVE_SDK_USB_PRODUCTID_0     3379 // mac
#define ODRIVE_SDK_USB_PRODUCTID_1     3378 // linux?

#define ODRIVE_SDK_PROTOCOL_VERION 1
#define ODRIVE_SDK_DEFAULT_CRC_VALUE 13145  // found with running expore_odrive -v and outputting my own information
#define ODRIVE_SDK_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_SDK_TIMEOUT 1000
#define ODRIVE_SDK_MAX_RESULT_LENGTH 100
#define ODRIVE_SDK_LIBUSB_ISERIAL_LENGTH 256

#define ODRIVE_SDK_SCAN_SUCCESS 0
#define ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND 1
#define ODRIVE_SDK_SERIAL_NUMBER_MAP_INVALID 2
#define ODRIVE_SDK_UNEXPECTED_RESPONSE 3
#define ODRIVE_SDK_NOT_INITIALIZED 4
#define ODRIVE_SDK_COMM_SUCCESS 0

#define ODRIVE_SDK_SERIAL_NUMBER_CMD 2

#define ODRIVE_SDK_WRITING_ENDPOINT 3 // found with running expore_odrive -v
#define ODRIVE_SDK_READING_ENDPOINT 131 // found with running expore_odrive -v

#define ODRIVE_SDK_SET_POS_0_CMD 125 // found with running expore_odrive -v
#define ODRIVE_SDK_SET_POS_1_CMD 268 // found with running expore_odrive -v

#define ODRIVE_SDK_SET_CURRENT_0_CMD 128
#define ODRIVE_SDK_SET_CURRENT_1_CMD 271

#define ODRIVE_SDK_GET_ENCODER_0_STATE 161 // found with running expore_odrive -v
#define ODRIVE_SDK_GET_ENCODER_1_STATE 304 // found with running expore_odrive -v

#define ODRIVE_SDK_GET_MOTOR_0_ERROR 77 // found with running expore_odrive -v
#define ODRIVE_SDK_GET_MOTOR_1_ERROR 220 // found with running expore_odrive -v

#define ODRIVE_SDK_REQUESTED_STATE_0_CMD 55
#define ODRIVE_SDK_REQUESTED_STATE_1_CMD 198

#define ODRIVE_SDK_CONTROL_MODE_0_CMD 131
#define ODRIVE_SDK_CONTROL_MODE_1_CMD 274

#define AXIS_STATE_UNDEFINED 0
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_STARTUP_SEQUENCE 2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3
#define AXIS_STATE_MOTOR_CALIBRATION 4
#define AXIS_STATE_SENSORLESS_CONTROL 5
#define AXIS_STATE_ENCODER_INDEX_SEARCH 6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define AXIS_STATE_LOCKIN_SPIN 9
#define AXIS_STATE_ENCODER_DIR_FIND 10
#define CTRL_MODE_CURRENT_CONTROL 1

#define ODRIVE_SDK_MOTOR_NO_ERROR_STATUS 0 // what odrive would return when no motor errors

namespace odrive
{

    class CppSdk {

    public:
        CppSdk(
               const std::string* odrive_serial_numbers,
               const uint8_t num_odrives,
               const std::string* motor_to_odrive_serial_number_map,
               const bool* motor_position_map, // false = slot 0, true = slot 1
               const float* encoder_ticks_per_radian,
               const bool* motor_relative_to_prior_motor, // true if there is influence, like a belt drive
               const uint8_t num_motors
               );
        ~CppSdk();

        int init(); // start communication
        int runCalibration();
        int allReady();
        int allIdle();
        int setCurrentCtrlMode();
        void setZeroethRadianInEncoderTicks(const int16_t* zeroeth_radian_in_encoder_ticks); // assumed to match num_motors
        int setGoalMotorPositions(const double* axes_positions_in_radians_array); // assumed to match num_motors
        int setCurrentSetpoint(const float* axes_current_in_A_array);
        int readMotorPositions(double* axes_positions_in_radians_array); // assumed to match num_motors
        int readEncoders(float* axes_positions_in_cpr_array);
        int checkErrors(uint8_t* error_codes_array); // assumed to match num_motors

    private:

        // read settings
        uint8_t num_odrives_;
        uint8_t num_motors_;
        float* encoder_ticks_per_radian_;
        int16_t* zeroeth_radian_in_encoder_ticks_;
        bool* motor_position_map_;
        bool* motor_relative_to_prior_motor_;

        // saved for use between creation and init
        std::string* odrive_serial_numbers_;
        std::string* motor_to_odrive_serial_number_map_;

        // for usb
        libusb_device_handle** odrive_handles_;
        libusb_context* libusb_context_;
        uint8_t* motor_to_odrive_handle_index_;
        int initUSBHandlesBySNs();

        short outbound_seq_no_; // unique ids for packets send to odrive

        int odriveEndpointRequest(libusb_device_handle* handle, int endpoint_id, commBuffer& received_payload, int& received_length, const commBuffer payload, const int ack, const int length);
        int odriveEndpointGetShort(libusb_device_handle* handle, int endpoint_id, short& value);
        int odriveEndpointGetInt(libusb_device_handle* handle, int endpoint_id, int& value);
        int odriveEndpointGetFloat(libusb_device_handle* handle, int endpoint_id, float& value);
        int odriveEndpointGetUInt16(libusb_device_handle* handle, int endpoint_id, uint16_t& value);
        int odriveEndpointGetUInt64(libusb_device_handle* handle, int endpoint_id, uint64_t& value);
        int odriveEndpointSetUInt8(libusb_device_handle* handle, int endpoint_id, const uint8_t& value);
        int odriveEndpointSetFloat(libusb_device_handle* handle, int endpoint_id, const float& value);
        void serializeCommBufferInt(commBuffer& buf, const int& value);
        void serializeCommBufferFloat(commBuffer& buf, const float& value);
        void deserializeCommBufferInt(commBuffer& buf, int& value);
        void deserializeCommBufferFloat(commBuffer& buf, float& value);
        void appendShortToCommBuffer(commBuffer& buf, const short value);
        void readShortFromCommBuffer(commBuffer& buf, short& value);
        void deserializeCommBufferUInt64(commBuffer& buf, uint64_t& value);
        void deserializeCommBufferUInt16(commBuffer& buf, uint16_t& value);

        commBuffer createODrivePacket(short seq_no, int endpoint, short response_size, const commBuffer& payload_ref);
        commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet);
    };
}

#endif /* ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_ */
