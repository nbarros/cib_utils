
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(test_bitmask EXCLUDE_FROM_ALL test_bitmask.cpp)
#target_compile_features(dac_ad5339_test PUBLIC cxx_std_11)
#set_target_properties(dac_ad5339_test PROPERTIES CXX_STANDARD_REQUIRED ON)
#set_target_properties(dac_ad5339_test PROPERTIES CXX_EXTENSIONS OFF)
#target_include_directories(dac_ad5339_test PRIVATE ${PROJECT_SOURCE_DIR}/dac)
#target_link_libraries(dac_ad5339_test i2c)

add_executable(test_signed_casting EXCLUDE_FROM_ALL test_signed_casting.cpp)
target_link_libraries(test_signed_casting PUBLIC common spdlog::spdlog)

