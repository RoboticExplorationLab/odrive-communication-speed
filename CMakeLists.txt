cmake_minimum_required(VERSION 3.13)
project(odrive_test)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS -pthread)

find_package(PkgConfig REQUIRED)

set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost 1.71.0 COMPONENTS date_time filesystem system ...)
include_directories(${Boost_INCLUDE_DIRS})

pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

include_directories(.)
include_directories(include)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
include_directories(lib)
include_directories(${LIBUSB1_INCLUDEDIR})
link_directories(${LIBUSB1_LIBRARY_DIRS})

add_executable(odrive_test tests/odrive_test.cpp)
add_executable(c_sdk_test tests/c_sdk_test.cpp)
add_executable(pid_test tests/pid_test.cpp)
add_executable(pid_watchdog_test tests/pid_watchdog_test.cpp)
add_executable(c_sdk_test_single_odrive tests/c_sdk_test_single_odrive.cpp)

add_library(odrive_cpp_sdk SHARED src/odrive_cpp_sdk.cpp)
add_library(odrive_c_sdk SHARED src/odrive_c_sdk.cpp src/odrive_threadpool.cpp)

install(TARGETS odrive_cpp_sdk DESTINATION lib)
install(TARGETS odrive_c_sdk DESTINATION lib)

target_link_libraries(odrive_cpp_sdk ${Boost_LIBRARIES})
target_link_libraries(odrive_cpp_sdk ${LIBUSB1_LIBRARIES})
target_link_libraries(odrive_c_sdk odrive_cpp_sdk)
target_link_libraries(odrive_test odrive_cpp_sdk ${LIBUSB1_LIBRARIES})
target_link_libraries(c_sdk_test odrive_c_sdk)
target_link_libraries(pid_test odrive_c_sdk)
target_link_libraries(pid_watchdog_test odrive_c_sdk)
target_link_libraries(c_sdk_test_single_odrive odrive_c_sdk)

install(TARGETS c_sdk_test DESTINATION bin)
install(TARGETS pid_test DESTINATION bin)
install(TARGETS c_sdk_test_single_odrive DESTINATION bin)
install(TARGETS odrive_test DESTINATION bin)