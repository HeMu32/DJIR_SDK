cmake_minimum_required(VERSION 3.13)



###############################################################################
## PROJECT
## name and version
###############################################################################
project(DJIR_SDK LANGUAGES CXX)

set(DJIR_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

if(NOT DEFINED RSCA_USBCAN_SDK_DIR)
	if(DEFINED RSCA_3RDPARTY_DIR AND EXISTS "${RSCA_3RDPARTY_DIR}/USBCAN_SDK/CMakeLists.txt")
		set(RSCA_USBCAN_SDK_DIR "${RSCA_3RDPARTY_DIR}/USBCAN_SDK")
	else()
		set(RSCA_USBCAN_SDK_DIR "${DJIR_SDK_ROOT}/3rdparty/USBCAN_SDK")
	endif()
endif()



###############################################################################
## INCLUDING SUBDIRECTORIES
## Adding subdirectories according to the configuration
###############################################################################
# 3rdparty
add_subdirectory(3rdparty)
# src
add_subdirectory(src)
# examples
# add_subdirectory(examples)
