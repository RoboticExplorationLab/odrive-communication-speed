cmake_minimum_required(VERSION 3.14)
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

add_executable(odrive_test src/main.cpp)

add_library(odrive_cpp_sdk SHARED src/odrive_cpp_sdk.cpp)
add_library(odrive_c_sdk SHARED src/odrive_c_sdk.cpp)

install(TARGETS odrive_cpp_sdk DESTINATION lib)
install(TARGETS odrive_c_sdk DESTINATION lib)

target_link_libraries(odrive_cpp_sdk ${Boost_LIBRARIES})
target_link_libraries(odrive_cpp_sdk ${LIBUSB1_LIBRARIES})
target_link_libraries(odrive_test odrive_cpp_sdk ${LIBUSB1_LIBRARIES})

target_link_libraries(odrive_c_sdk odrive_cpp_sdk)

install(TARGETS odrive_test DESTINATION bin)