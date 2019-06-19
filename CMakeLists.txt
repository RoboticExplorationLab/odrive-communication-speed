cmake_minimum_required(VERSION 3.14)
project(odrive_test)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

include_directories(.)
include_directories(include)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
include_directories(lib)
include_directories(${LIBUSB1_INCLUDEDIR})
link_directories(${LIBUSB1_LIBRARY_DIRS})

add_executable(odrive_test src/main.cpp)

add_library(odrive_cpp_sdk src/odrive_cpp_sdk.cpp)
install(TARGETS odrive_cpp_sdk DESTINATION lib)

target_link_libraries(odrive_cpp_sdk ${LIBUSB1_LIBRARIES})

target_link_libraries(odrive_test odrive_cpp_sdk ${LIBUSB1_LIBRARIES})

install(TARGETS odrive_test DESTINATION bin)