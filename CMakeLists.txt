cmake_minimum_required(VERSION 3.2)

# Set some basic project attributes
project (cib_tools
	VERSION 0.1
	DESCRIPTION "A Hello World Project")


add_subdirectory(versaclock)
add_subdirectory(pdts)


# This project will output an executable file
#add_executable(${PROJECT_NAME} cib_tools.cpp)

# Create a simple configuration header
#configure_file(config.h.in config.h)

# Include the configuration header in the build
#target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
