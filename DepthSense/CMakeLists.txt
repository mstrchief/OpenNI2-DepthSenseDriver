cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(DepthSenseDevice)

INCLUDE(FindPkgConfig)
pkg_check_modules (LIBUSB REQUIRED libusb-1.0)
include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})

set(DEPTHSENSESDK_DIR /opt/softkinetic/DepthSenseSDK)
set(DEPTHSENSESDK_INCLUDE_DIR ${DEPTHSENSESDK_DIR}/include)
set(DEPTHSENSESDK_LIBRARY_DIR ${DEPTHSENSESDK_DIR}/lib)
set(DEPTHSENSESDK_LIBRARIES ${DEPTHSENSESDK_LIBRARY_DIR}/libDepthSense.so ${DEPTHSENSESDK_LIBRARY_DIR}/libDepthSensePlugins.so ${DEPTHSENSESDK_LIBRARY_DIR}/libturbojpeg.so ${DEPTHSENSESDK_LIBRARY_DIR}/libusb-1.0.so)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(OPENNI2_XNLIB ${CMAKE_SOURCE_DIR}/../thirdparty/lib/libXnLib.a)

include_directories(${CMAKE_SOURCE_DIR}/../thirdparty/include/)

include_directories(${DEPTHSENSESDK_INCLUDE_DIR})
link_directories(${DEPTHSENSESDK_LIBRARY_DIR})

#add_executable (DepthSenseDevice
#	DepthSenseDevice.cpp
#)
add_library (DepthSenseDevice SHARED
	DepthSenseDevice.cpp
)
target_link_libraries (${PROJECT_NAME} ${SSE_FLAGS} ${OPEN_NI_LIBRARIES} ${OPENNI2_XNLIB})
