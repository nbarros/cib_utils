cmake_minimum_required(VERSION 3.2)

# Set some basic project attributes
project (cib_tools
	VERSION 0.1
	DESCRIPTION "A Hello World Project")

add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

# Do not request this. Instead use the one out of contrib
find_package(spdlog REQUIRED)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/contrib/spdlog/include)
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/contrib/cppzmq)

file(GLOB COMMON_SRC_FILES ${PROJECT_SOURCE_DIR}/common/*.cpp)
add_library(common SHARED ${COMMON_SRC_FILES})
target_compile_features(common PUBLIC cxx_std_11)
set_target_properties(common PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(common PROPERTIES CXX_EXTENSIONS OFF)

#set(SPDLOG_BUILD_PIC ON CACHE BOOL "enable spdlog PIC code")
#add_subdirectory(contrib/spdlog)
# do not compile 
#add_subdirectory(contrib/cppzmq)

add_subdirectory(versaclock)
add_subdirectory(pdts)

add_subdirectory(dac)
add_subdirectory(axi_fifo)
add_subdirectory(i2c)
add_subdirectory(cib_debug)
add_subdirectory(apps)
add_subdirectory(tests)


# This project will output an executable file
#add_executable(${PROJECT_NAME} cib_tools.cpp)

# Create a simple configuration header
#configure_file(config.h.in config.h)

# Include the configuration header in the build
#target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
