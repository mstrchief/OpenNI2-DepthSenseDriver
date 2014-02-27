cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(DepthSenseDevice)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror -Wno-gnu-static-float-init -Wno-unused-function -fPIC")

set(DEPTHSENSESDK_DIR /opt/softkinetic/DepthSenseSDK)
set(DEPTHSENSESDK_INCLUDE_DIR ${DEPTHSENSESDK_DIR}/include)
set(DEPTHSENSESDK_LIBRARY_DIR ${DEPTHSENSESDK_DIR}/lib)
set(DEPTHSENSESDK_LIBRARIES ${DEPTHSENSESDK_LIBRARY_DIR}/libDepthSense.so ${DEPTHSENSESDK_LIBRARY_DIR}/libDepthSensePlugins.so ${DEPTHSENSESDK_LIBRARY_DIR}/libturbojpeg.so ${DEPTHSENSESDK_LIBRARY_DIR}/libusb-1.0.so)
include_directories(${DEPTHSENSESDK_INCLUDE_DIR})
link_directories(${DEPTHSENSESDK_LIBRARY_DIR})

set(OPENNI2_XNLIB ${CMAKE_SOURCE_DIR}/../thirdparty/lib/libXnLib.a)
include_directories(${CMAKE_SOURCE_DIR}/../thirdparty/include/)

add_library (DepthSenseDevice SHARED
	DepthSenseDevice.cpp
)

target_link_libraries (${PROJECT_NAME} ${DEPTHSENSESDK_LIBRARIES} ${OPENNI2_XNLIB} dl rt pthread udev usb-1.0 freenect)
