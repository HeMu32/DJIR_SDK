cmake_minimum_required(VERSION 3.13)

###############################################################################
## 3RD-PARTY
## dependencies for the project
###############################################################################
project(3rdparty LANGUAGES CXX)

# RSCA_USBCAN_SDK_DIR is resolved by the parent DJIR_SDK/CMakeLists.txt.
if(NOT EXISTS "${RSCA_USBCAN_SDK_DIR}/CMakeLists.txt")
	message(FATAL_ERROR "[DJIR_SDK] Missing required dependency: ${RSCA_USBCAN_SDK_DIR}/CMakeLists.txt")
endif()



###############################################################################
## INCLUDING SUBDIRECTORIES
## Adding subdirectories according to the 3rd-party configuration
###############################################################################
add_subdirectory("${RSCA_USBCAN_SDK_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/USBCAN_SDK")
add_subdirectory(custom_crc)




