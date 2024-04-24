

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(cib_i2c SHARED Device.cpp ADN2814.cpp AD5339.cpp)
#add_executable(program_vc program_vc.c)
target_include_directories(cib_i2c PRIVATE ${PROJECT_SOURCE_DIR}/i2c)

add_executable(test_struct test_struct_msb.cpp)

add_executable(check_cdr_chip check_cdr_chip.cpp)
target_include_directories(check_cdr_chip PRIVATE ${PROJECT_SOURCE_DIR}/i2c)
target_link_libraries(check_cdr_chip cib_i2c readline i2c)

add_executable(dac_manager dac_manager.cpp)
target_include_directories(dac_manager PRIVATE ${PROJECT_SOURCE_DIR}/i2c)
target_link_libraries(dac_manager cib_i2c readline i2c)