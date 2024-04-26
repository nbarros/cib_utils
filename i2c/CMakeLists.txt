

set(CMAKE_VERBOSE_MAKEFILE ON)

#find_package(spdlog REQUIRED)
#find_package(i2c REQUIRED)


add_library(cib_i2c SHARED Device.cpp ADN2814.cpp AD5339.cpp)
#add_executable(program_vc program_vc.c)
target_include_directories(cib_i2c PRIVATE ${PROJECT_SOURCE_DIR}/i2c)
target_link_libraries(cib_i2c i2c spdlog::spdlog)

add_executable(test_struct test_struct_msb.cpp)

add_executable(cdr_manager cdr_manager.cpp)
target_include_directories(cdr_manager PRIVATE ${PROJECT_SOURCE_DIR}/i2c)
target_link_libraries(cdr_manager cib_i2c readline i2c spdlog)

add_executable(dac_manager dac_manager.cpp)
target_include_directories(dac_manager PRIVATE ${PROJECT_SOURCE_DIR}/i2c)
target_link_libraries(dac_manager cib_i2c readline i2c spdlog)
