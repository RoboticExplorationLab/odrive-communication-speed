# ODrive Communication Library
### A fast C++ library for running closed-loop control of multiple ODrives connected over USB.

## Setup
- So far we've only had luck connecting to the ODrives on Linux computers because for some reason, libusb can't establish a connection with the ODrives on macOS, and Windows is just out of the question.
- Compile dynamic shared libraries by running these commands:
```bash
mkdir build; cd build
cmake ..
make
```
## Testing
- Change directory to the ```build``` directory and then run the test scripts.
    - The executable ```odrive_test``` tests the primary C++ library by sending current commands to the ODrives and receiving the incoming encoder data for several seconds.
    - The exectuable ```c_sdk_test``` tests the C-style wrapper for the primary library by sending a current command and reading the resultant encoder measurement.
```bash
cd build
sudo ./odrive_test
sudo ./c_sdk_test
```