#include "odrive_cpp_sdk.h"
#include <cstring>
#include <algorithm>
#include <typeinfo>
#include <string>
#include <boost/endian/conversion.hpp>

using namespace odrive;

CppSdk::CppSdk(const std::string& odrive_serial_number,
               const bool* motor_position_map, // false = slot 0, true = slot 1
               const float* encoder_ticks_per_radian,
               const bool* motor_relative_to_prior_motor, // false = motors do not influence eachother, true = this motor poisition is subtracted from the prior one
               const uint8_t num_motors) {
    // read settings
    num_motors_ = num_motors;
    encoder_ticks_per_radian_ = new float[num_motors]();
    for (uint8_t i = 0; i < num_motors; ++i) {
        encoder_ticks_per_radian_[i] = encoder_ticks_per_radian[i];
    }
    zeroeth_radian_in_encoder_ticks_ = new int16_t[num_motors]();
    for (uint8_t i = 0; i < num_motors; ++i) {
        zeroeth_radian_in_encoder_ticks_[i] = 0;
    }
    motor_position_map_ = new bool[num_motors]();
    for (uint8_t i = 0; i < num_motors; ++i) {
        motor_position_map_[i] = motor_position_map[i];
    }
    motor_relative_to_prior_motor_ = new bool[num_motors]();
    for (uint8_t i = 0; i < num_motors; ++i) {
        motor_relative_to_prior_motor_[i] = motor_relative_to_prior_motor[i];
    }

    // saved for use between creation and init
    odrive_serial_number_ = odrive_serial_number;

    libusb_context_ = NULL;
}

CppSdk::~CppSdk() {
    if (odrive_handle_) {
        int result = libusb_release_interface(odrive_handle_, 0);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Error calling libusb_release_interface on odrive `" << odrive_serial_number_ << "`: " << result << " - " << libusb_error_name(result) << std::endl;
            std::cerr << "Error calling libusb_release_interface on odrive `" << odrive_serial_number_ << "`: " << result << " - " << libusb_error_name(result) << std::endl;
        }

        libusb_close(odrive_handle_);
        odrive_handle_ = NULL;
    }

    delete [] encoder_ticks_per_radian_;
    delete [] zeroeth_radian_in_encoder_ticks_;
    delete [] motor_position_map_;
    delete [] motor_relative_to_prior_motor_;

    // usb
    if (libusb_context_) { libusb_exit(libusb_context_); }
}

int CppSdk::init() {
    if (libusb_context_ != NULL) {
      std::cerr << "ODrive SDK's init function has been called twice." << std::endl;
      return ODRIVE_SDK_NOT_INITIALIZED;
    }

    outbound_seq_no_ = 0;
    // for USB
    int result = libusb_init(&libusb_context_);
    if (result != LIBUSB_SUCCESS) {
        return result; // error message should have been printed before
    }

    result = initUSBHandlesBySNs();
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result; // error message should have been printed before
    }
    if (!odrive_handle_) {
        return ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND;
    }

    was_init_ = true;

    return ODRIVE_SDK_COMM_SUCCESS;
}

void CppSdk::setZeroethRadianInEncoderTicks(const int16_t* zeroeth_radian_in_encoder_ticks) {
    for (uint8_t i = 0; i < num_motors_; ++i) {
        zeroeth_radian_in_encoder_ticks_[i] = zeroeth_radian_in_encoder_ticks[i];
    }
}

int CppSdk::runCalibration(){
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i){
        cmd = motor_position_map_[i] ? ODRIVE_SDK_REQUESTED_STATE_1_CMD : ODRIVE_SDK_REQUESTED_STATE_0_CMD;

        int result = odriveEndpointSetUInt8(odrive_handle_, cmd, AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't run full calibration " << std::to_string(cmd) << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::getRequestedState(int cmd, uint8_t& state){
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int result = odriveEndpointGetUInt8(odrive_handle_, cmd, state);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Couldn't run full calibration " << std::to_string(cmd) << "': `" << result << "` (see prior error message)" << std::endl;
        return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::allReady(){
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i){
        cmd = motor_position_map_[i] ? ODRIVE_SDK_REQUESTED_STATE_1_CMD : ODRIVE_SDK_REQUESTED_STATE_0_CMD;

        int result = odriveEndpointSetUInt8(odrive_handle_, cmd, AXIS_STATE_CLOSED_LOOP_CONTROL);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't set to closed loop " << std::to_string(cmd) << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::allIdle(){
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i){
        cmd = motor_position_map_[i] ? ODRIVE_SDK_REQUESTED_STATE_1_CMD : ODRIVE_SDK_REQUESTED_STATE_0_CMD;

        int result = odriveEndpointSetUInt8(odrive_handle_, cmd, AXIS_STATE_IDLE);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't set to idle " << std::to_string(cmd) << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::setCurrentCtrlMode(){
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i){
        cmd = motor_position_map_[i] ? ODRIVE_SDK_CONTROL_MODE_0_CMD : ODRIVE_SDK_CONTROL_MODE_1_CMD;

        int result = odriveEndpointSetUInt8(odrive_handle_, cmd, CTRL_MODE_CURRENT_CONTROL);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't set to current control " << std::to_string(cmd) << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::setGoalMotorPositions(const double* axes_positions_in_radians_array) {
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        double target_ticks = axes_positions_in_radians_array[i] * encoder_ticks_per_radian_[i];
        if (i != 0 && motor_relative_to_prior_motor_[i]) {
            target_ticks = (axes_positions_in_radians_array[i] - axes_positions_in_radians_array[i-1] /* * encoder_ticks_per_radian_[i-1] */) * encoder_ticks_per_radian_[i];
        }
        float position_in_ticks = (int) (zeroeth_radian_in_encoder_ticks_[i] + target_ticks);

        cmd = motor_position_map_[i] ? ODRIVE_SDK_SET_POS_1_CMD : ODRIVE_SDK_SET_POS_0_CMD;

        int result = odriveEndpointSetFloat(odrive_handle_, cmd, position_in_ticks);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't send `" << std::to_string(cmd) << " " << std::to_string(position_in_ticks) << "` to '" << odrive_serial_number_ << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }

    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::setCurrentSetpoint(const float* axes_current_in_A_array) {
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i) {

        cmd = motor_position_map_[i] ? ODRIVE_SDK_SET_CURRENT_1_CMD : ODRIVE_SDK_SET_CURRENT_0_CMD;

        int result = odriveEndpointSetFloat(odrive_handle_, cmd, axes_current_in_A_array[i]);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't send `" << std::to_string(cmd) << " " << std::to_string(axes_current_in_A_array[i]) << "` to '" << odrive_serial_number_ << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }

    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::readMotorPositions(double* axes_positions_in_radians_array) {
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        cmd = motor_position_map_[i] ? ODRIVE_SDK_GET_ENCODER_1_STATE : ODRIVE_SDK_GET_ENCODER_0_STATE;

        int read_encoder_ticks;
        int result = odriveEndpointGetInt(odrive_handle_, cmd, read_encoder_ticks);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_number_ << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }

        // NOTE!  THIS RELIES ON THE AXIS POSITIONS BEING READ IN ORDER; THAT axes_positions_in_radians_array[i-1] WAS READ FROM ODRIVE ALREADY
        double interpreted_radians = read_encoder_ticks / (double)encoder_ticks_per_radian_[i];
        if (i != 0 && motor_relative_to_prior_motor_[i]) {
          interpreted_radians = axes_positions_in_radians_array[i-1] + (read_encoder_ticks / (double)encoder_ticks_per_radian_[i]);
        }

        axes_positions_in_radians_array[i] =  interpreted_radians - (zeroeth_radian_in_encoder_ticks_[i] / (double)encoder_ticks_per_radian_[i]); // TODO Check math
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::readEncoders(float* axes_positions_in_cpr_array) {
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        cmd = motor_position_map_[i] ? ODRIVE_SDK_GET_ENCODER_1_STATE : ODRIVE_SDK_GET_ENCODER_0_STATE;

        float read_encoder_ticks;
        int result = odriveEndpointGetFloat(odrive_handle_, cmd, read_encoder_ticks);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_number_ << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }

        axes_positions_in_cpr_array[i] =  read_encoder_ticks;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::useTestFunction(int in){
    int cmd = ODRIVE_SDK_TEST_ARG;

    odriveEndpointSetInt(odrive_handle_, cmd, in);

    cmd = ODRIVE_SDK_TEST_FUNC;
    odriveEndpointSetInt(odrive_handle_, cmd, in);

    cmd = ODRIVE_SDK_TEST_OUT;

    int res;
    odriveEndpointGetInt(odrive_handle_, cmd, res);

    return res;
}

float CppSdk::getEncodersFunction(float current0) {
    int cmd = ODRIVE_SDK_GET_ENCODERS_ARG;
    odriveEndpointSetFloat(odrive_handle_, cmd, current0);

    cmd = ODRIVE_SDK_GET_ENCODERS_FUNC;
    odriveEndpointSetFloat(odrive_handle_, cmd, 0);

    cmd = ODRIVE_SDK_GET_ENCODERS_OUT;
    float read_encoder_ticks;
    odriveEndpointGetFloat(odrive_handle_, cmd, read_encoder_ticks);

    return read_encoder_ticks;
}

int CppSdk::getEncodersStructFunction(const current_command_t& current_cmd, encoder_measurements_t& encoder_meas) {
    int result;
    int cmd = ODRIVE_SDK_GET_ENCODERS_ARG;
    result = odriveEndpointSetCurrentCmd(odrive_handle_, current_cmd);

    cmd = ODRIVE_SDK_GET_ENCODERS_STRUCT_FUNC;
    result = odriveEndpointSetFloat(odrive_handle_, cmd, 0);

    result = odriveEndpointGetEncoderMeas(odrive_handle_, encoder_meas);

    return result;
}

int CppSdk::checkErrors(uint8_t* error_codes_array) {
    if (! was_init_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        cmd = motor_position_map_[i] ? ODRIVE_SDK_GET_MOTOR_1_ERROR : ODRIVE_SDK_GET_MOTOR_0_ERROR;
        uint16_t motor_error_output;
        int result = odriveEndpointGetUInt16(odrive_handle_, cmd, motor_error_output);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "CppSdk::checkErrors couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_number_ << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
        error_codes_array[i] = motor_error_output;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}


int CppSdk::initUSBHandlesBySNs() {
    libusb_device ** usb_device_list;
    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);
    if (device_count <= 0) {
        std::cerr << "Could not call libusb_get_device_list: " << device_count << " - " << libusb_error_name(device_count) << std::endl;
        return device_count;
    }

    for (size_t i = 0; i < device_count; ++i) {
        libusb_device *device = usb_device_list[i];
        libusb_device_descriptor desc = {0};

        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Could not call libusb_get_device_descriptor: " << result << " - " << libusb_error_name(result) << std::endl;
        }

        if (desc.idVendor == ODRIVE_SDK_USB_VENDORID && ( desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_0 ||  desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_1 )) {
            libusb_device_handle *device_handle;
            result = libusb_open(device, &device_handle);
            if (result != LIBUSB_SUCCESS) {
                std::cerr << "Could not call libusb_open: " << result << " - " << libusb_error_name(result) << std::endl;
            } else if (libusb_kernel_driver_active(device_handle, 0) && ( (result = libusb_detach_kernel_driver(device_handle, 0)) != LIBUSB_SUCCESS )) { // detach kernel driver if necessary
                std::cerr << "Could not call libusb_detach_kernel_driver: " << result << " - " << libusb_error_name(result) << std::endl;
            }
            else if ( (result = libusb_claim_interface(device_handle, 0)) !=  LIBUSB_SUCCESS ) {
              std::cerr << "Could not call libusb_claim_interface: " << result << " - " << libusb_error_name(result) << ": " << strerror(errno) << std::endl;
              libusb_close(device_handle);
            }
            else {
                bool attached_to_handle = false;
                uint64_t read_serial_number = 0;
                int result = odriveEndpointGetUInt64(device_handle, ODRIVE_SDK_SERIAL_NUMBER_CMD, read_serial_number);
                if (result != LIBUSB_SUCCESS) {
                    std::cerr << "Couldn't send `" << std::to_string(ODRIVE_SDK_SERIAL_NUMBER_CMD) << "` to '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                } else {
                    // find the right index for it in odrive_handle_
                    if (0 == odrive_serial_number_.compare(std::to_string(read_serial_number))) { // found!  no need to close, but need to free device list
                        odrive_handle_ = device_handle;
                        attached_to_handle = true;
                        break;
                    }
                }

                if (!attached_to_handle) {
                    std::cerr << "Odrive with serial number " << std::to_string(read_serial_number) << " but not requested; ignoring odrive." << std::endl;
                    result = libusb_release_interface(device_handle, 0);
                    if (result != LIBUSB_SUCCESS) {
                        std::cerr << "Error calling libusb_release_interface on '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                    }
                    libusb_close(device_handle);
                } else{
					std::cout << "Matched ODrive: " << std::to_string(read_serial_number) << std::endl;
				}
            }
        }
    }

    libusb_free_device_list(usb_device_list, 1);
    return ODRIVE_SDK_COMM_SUCCESS;
}


int CppSdk::odriveEndpointRequest(libusb_device_handle* handle, int endpoint_id, commBuffer& received_payload, int& received_length, commBuffer payload, int ack, int length) {
    commBuffer send_buffer;
    commBuffer receive_buffer;
    unsigned char receive_bytes[ODRIVE_SDK_MAX_RESULT_LENGTH] = { 0 };
    int sent_bytes = 0;
    int received_bytes = 0;
    short received_seq_no = 0;

    if (ack) {
        endpoint_id |= 0x8000;
    }
    outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
    outbound_seq_no_ |= 0x80;
    short seq_no = outbound_seq_no_;

    // Send the packet
    commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, payload);
	/*
    for(int i = 0; i<packet.size(); i++){
        std::cout << "i is: " << i << " - " << std::hex << (int)packet[i] << std::endl;
    } */

    int result = libusb_bulk_transfer(handle, ODRIVE_SDK_WRITING_ENDPOINT, packet.data(), packet.size(), &sent_bytes, 0);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Could not call libusb_bulk_transfer for writing: " << result << " - " << libusb_error_name(result) << strerror(errno) << std::endl;
        return result;
    } else if (packet.size() != sent_bytes) {
        std::cerr << "Could not call libusb_bulk_transfer: only wrote " << std::to_string(sent_bytes) << " of " << std::to_string(packet.size()) << " bytes (wanted to send `" << packet.data() << "`)" << std::endl;
    }

    if (ack) {
      // Immediatly wait for response from Odrive and check if ack (if we asked for one)
      result = libusb_bulk_transfer(handle, ODRIVE_SDK_READING_ENDPOINT, receive_bytes, ODRIVE_SDK_MAX_BYTES_TO_RECEIVE, &received_bytes, ODRIVE_SDK_TIMEOUT);
      if (result != LIBUSB_SUCCESS) {
          std::cerr << "Could not call libusb_bulk_transfer for reading: " << result << " - " << libusb_error_name(result) << strerror(errno) << std::endl;
          return result;
      }

      for (int i = 0; i < received_bytes; i++) {
          receive_buffer.push_back(receive_bytes[i]);
      }

      received_payload = decodeODrivePacket(receive_buffer, received_seq_no, receive_buffer);

	  /*
	  for(int i = 0; i<received_payload.size(); i++){
		std::cout << "j is: " << i << " - " << std::hex << (int)received_payload[i] << std::endl;
		}	 */

      if (received_seq_no != seq_no) {
          std::cerr << "[ERROR] Recieved packet from odrive for sequence number " << std::to_string(received_seq_no) << " but was expecting to recieve reply for sequence number " << std::to_string(seq_no) << ".  Sequence ordering is not implemented yet!!!" << std::endl;
      }

      // return the response payload
      received_length = received_payload.size();
    }

    return LIBUSB_SUCCESS;
}

int CppSdk::odriveEndpointGetUInt16(libusb_device_handle* handle, int endpoint_id, uint16_t& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 2);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    deserializeCommBufferUInt16(receive_payload, value);

    return LIBUSB_SUCCESS;
}

int CppSdk::odriveEndpointGetShort(libusb_device_handle* handle, int endpoint_id, short& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 2);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    readShortFromCommBuffer(receive_payload, value);

    return LIBUSB_SUCCESS;
}
int CppSdk::odriveEndpointGetInt(libusb_device_handle* handle, int endpoint_id, int& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 4);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    deserializeCommBufferInt(receive_payload, value);
    return LIBUSB_SUCCESS;
}

int CppSdk::odriveEndpointGetFloat(libusb_device_handle* handle, int endpoint_id, float& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 4);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    deserializeCommBufferFloat(receive_payload, value);
    return LIBUSB_SUCCESS;
}

int CppSdk::odriveEndpointGetUInt64(libusb_device_handle* handle, int id, uint64_t& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, id, receive_payload, received_length, send_payload, 1, 8);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }
    deserializeCommBufferUInt64(receive_payload, value);
    return LIBUSB_SUCCESS;
}

int CppSdk::odriveEndpointGetUInt8(libusb_device_handle* handle, int id, uint8_t& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, id, receive_payload, received_length, send_payload, 1, 8);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }
    deserializeCommBufferUInt8(receive_payload, value);
    return LIBUSB_SUCCESS;
}

int CppSdk::odriveEndpointSetUInt8(libusb_device_handle* handle, int endpoint_id, const uint8_t& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferUInt8(send_payload, value);
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 0, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::odriveEndpointSetFloat(libusb_device_handle* handle, int endpoint_id, const float& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferFloat(send_payload, value);
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 0, 0); // ack was 1 before?
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::odriveEndpointSetInt(libusb_device_handle* handle, int endpoint_id, const int& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferInt(send_payload, value);
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 0, 0); // ack was 1 before?
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::odriveEndpointSetCurrentCmd(libusb_device_handle* handle, const current_command_t& value){
    commBuffer send_payload;
    commBuffer send_payload_2;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferFloat(send_payload, value.current_axis0);
    serializeCommBufferFloat(send_payload_2, value.current_axis1);

    send_payload.insert(send_payload.end(), send_payload_2.begin(), send_payload_2.end());

    int result = odriveEndpointRequest(handle, ODRIVE_SDK_GET_ENCODERS_STRUCT_ARG, receive_payload, received_length, send_payload, 0, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::odriveEndpointGetEncoderMeas(libusb_device_handle* handle, encoder_measurements_t& value){
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, ODRIVE_SDK_GET_ENCODERS_STRUCT_OUT, receive_payload,
            received_length, send_payload, 1, 4*sizeof(float));
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }

    if (received_length >= sizeof(float)*4) {
        float pos_axis0, vel_axis0, pos_axis1, vel_axis1;
        commBuffer receive_payload_1(receive_payload.begin(), receive_payload.begin() + sizeof(float));
        deserializeCommBufferFloat(receive_payload_1, pos_axis0);

        commBuffer receive_payload_2(receive_payload.begin() + sizeof(float),
                                     receive_payload.begin() + 2 * sizeof(float));
        deserializeCommBufferFloat(receive_payload_2, vel_axis0);

        commBuffer receive_payload_3(receive_payload.begin() + 2 * sizeof(float),
                                     receive_payload.begin() + 3 * sizeof(float));
        deserializeCommBufferFloat(receive_payload_3, pos_axis1);

        commBuffer receive_payload_4(receive_payload.begin() + 3 * sizeof(float),
                                     receive_payload.begin() + 4 * sizeof(float));
        deserializeCommBufferFloat(receive_payload_4, vel_axis1);

        value.encoder_pos_axis0 = pos_axis0;
        value.encoder_vel_axis0 = vel_axis0;
        value.encoder_pos_axis1 = pos_axis1;
        value.encoder_vel_axis1 = vel_axis1;
        return ODRIVE_SDK_COMM_SUCCESS;
    }
    else{
        std::cout << "Received length is too short to decode encoder struct" << std::endl;
        return -1;
    }
}

void CppSdk::appendShortToCommBuffer(commBuffer& buf, const short value) {
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
}


void CppSdk::readShortFromCommBuffer(commBuffer& byte_array, short& value) {
  //TODO: Check that -ve values are being converted correctly.
  value = 0;
  for(int i = 0; i < sizeof(short); ++i) {
     value <<= 8;
     value |= byte_array[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = boost::endian::endian_reverse(value);
}


void CppSdk::serializeCommBufferFloat(commBuffer& buf, const float& value) {
    for(int i = 0; i < sizeof(float); i++){
       buf.push_back(((unsigned char*)&value)[i]);
    }
}

void CppSdk::serializeCommBufferInt(commBuffer& buf, const int& value) {
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
    buf.push_back((value >> 16) & 0xFF);
    buf.push_back((value >> 24) & 0xFF);
}

void CppSdk::serializeCommBufferUInt8(commBuffer& buf, const uint8_t& value) {
    buf.push_back(value);
}

void CppSdk::deserializeCommBufferInt(commBuffer& byte_array, int& value) {
  //TODO: Check that -ve values are being converted correctly.
  value = 0;
  for(int i = 0; i < byte_array.size(); ++i) {
     value <<= 8;
     value |= byte_array[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = boost::endian::endian_reverse(value);
}

void CppSdk::deserializeCommBufferFloat(commBuffer& byte_array, float& value) {
    union {
        float f;
        int i;
    }u;
    deserializeCommBufferInt(byte_array, u.i);
    value = u.f;
}

void CppSdk::deserializeCommBufferUInt64(commBuffer& v, uint64_t& value) {
  value = 0;
  for(int i = 0; i < v.size(); ++i) {
     value <<= 8;
     value |= v[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = boost::endian::endian_reverse(value);
}

void CppSdk::deserializeCommBufferUInt16(commBuffer& v, uint16_t& value) {
  value = 0;
  for(int i = 0; i < v.size(); ++i) {
     value <<= 8;
     value |= v[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = boost::endian::endian_reverse(value);
}

void CppSdk::deserializeCommBufferUInt8(commBuffer& v, uint8_t& value) {
    value = 0;
//    for(int i = 0; i < v.size(); ++i) {
//        value <<= 8;
//        value |= v[i];
//    }
    value |= v[0];

    //Convert the byte array to little endian. It's currently being read in as a bigendian.
    value = boost::endian::endian_reverse(value);
}

commBuffer CppSdk::createODrivePacket(short seq_no, int endpoint_id, short response_size, const commBuffer& input) {
    commBuffer packet;
    short crc = 0;
    if ((endpoint_id & 0x7fff) == 0) {
        crc = ODRIVE_SDK_PROTOCOL_VERION;
    }
    else {
        crc = ODRIVE_SDK_DEFAULT_CRC_VALUE;
    }
	
	/*
	std::cout << "seq_no: " << std::hex << seq_no << std::endl;
	std::cout << "endpoint: " << std::hex << endpoint_id << std::endl;
	std::cout << "response_size: " << std::hex << response_size << std::endl;
	std::cout << "crc: " << std::hex << crc << std::endl;
	*/

    appendShortToCommBuffer(packet, seq_no);
    appendShortToCommBuffer(packet, endpoint_id);
    appendShortToCommBuffer(packet, response_size);

    for (uint8_t b : input) {
        packet.push_back(b);
    }

    appendShortToCommBuffer(packet, crc);

    return packet;
}

commBuffer CppSdk::decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet) {
    commBuffer payload;
    readShortFromCommBuffer(buf, seq_no); // reads 2 bytes so start next for loop at 2
    seq_no &= 0x7fff;
    for (commBuffer::size_type i = 2; i < buf.size(); ++i) {
        payload.push_back(buf[i]);
    }
    return payload;
}
