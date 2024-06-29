cmake_minimum_required(VERSION 3.2)

# Set some basic project attributes
project (cib_tools
	VERSION 1.0
	DESCRIPTION "CIB Utilities Project")

#add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

# Do not request this. Instead use the one out of contrib
find_package(spdlog REQUIRED)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/common)
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/contrib/cppzmq)


#set(SPDLOG_BUILD_PIC ON CACHE BOOL "enable spdlog PIC code")
#add_subdirectory(contrib/spdlog)
# do not compile 
#add_subdirectory(contrib/cppzmq)
#include_directories(${PROJECT_SOURCE_DIR}/contrib/spdlog/include)

add_subdirectory(common)
add_subdirectory(versaclock)
add_subdirectory(pdts)

add_subdirectory(dac)
add_subdirectory(axi_fifo)
add_subdirectory(i2c)
add_subdirectory(cib_debug)
add_subdirectory(daq)
add_subdirectory(apps)
add_subdirectory(tests)


# This project will output an executable file
#add_executable(${PROJECT_NAME} cib_tools.cpp)

# Create a simple configuration header
#configure_file(config.h.in config.h)

# Include the configuration header in the build
#target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
