cmake_minimum_required(VERSION 3.13)



###############################################################################
## 3RD-PARTY
## dependencies for the project
###############################################################################
project(3rdparty LANGUAGES CXX)

if(NOT DEFINED RSCA_USBCAN_SDK_DIR)
	set(RSCA_USBCAN_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/USBCAN_SDK")
endif()

if(NOT EXISTS "${RSCA_USBCAN_SDK_DIR}/CMakeLists.txt")
	message(FATAL_ERROR "[DJIR_SDK] Missing required dependency: ${RSCA_USBCAN_SDK_DIR}/CMakeLists.txt")
endif()



###############################################################################
## INCLUDING SUBDIRECTORIES
## Adding subdirectories according to the 3rd-party configuration
###############################################################################
add_subdirectory("${RSCA_USBCAN_SDK_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/USBCAN_SDK")
add_subdirectory(custom_crc)




