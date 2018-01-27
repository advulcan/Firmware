include(configs/posix_rpi_common)

add_definitions(
	-D __DF_RPI
	)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-lwiringPi")
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)
