
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(dac_ad5339_test dac_ad5339_test.cc)
#target_include_directories(check_timing ${PROJECT_SOURCE_DIR}/common)
target_compile_features(dac_ad5339_test PUBLIC cxx_std_11)
set_target_properties(dac_ad5339_test PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(dac_ad5339_test PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(dac_ad5339_test PRIVATE ${PROJECT_SOURCE_DIR}/dac)
target_link_libraries(dac_ad5339_test i2c)

add_executable(example_program_i2c example_program_i2c.cc)
target_include_directories(example_program_i2c PRIVATE ${PROJECT_SOURCE_DIR}/dac)
target_link_libraries(example_program_i2c i2c)


add_executable(dac_setup dac_setup.cc)
target_include_directories(dac_setup PRIVATE ${PROJECT_SOURCE_DIR}/dac)
target_link_libraries(dac_setup i2c)


#add_executable(check_timing check_timing.cpp)
##target_include_directories(check_timing ${PROJECT_SOURCE_DIR}/common)
#target_compile_features(check_timing PUBLIC cxx_std_11)
#set_target_properties(check_timing PROPERTIES CXX_STANDARD_REQUIRED ON)
#set_target_properties(check_timing PROPERTIES CXX_EXTENSIONS OFF)
#target_include_directories(check_timing PRIVATE ${PROJECT_SOURCE_DIR}/pdts)
#target_link_libraries(check_timing common)
