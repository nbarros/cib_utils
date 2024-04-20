

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(cibi2c SHARED Device.cpp ADN2814.cpp)
#add_executable(program_vc program_vc.c)
target_include_directories(cibi2c PRIVATE ${PROJECT_SOURCE_DIR}/i2c_misc)

add_executable(test_struct test_struct_msb.cpp)