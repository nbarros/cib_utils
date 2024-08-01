
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(test_bitmask EXCLUDE_FROM_ALL test_bitmask.cpp)
#target_compile_features(dac_ad5339_test PUBLIC cxx_std_11)
#set_target_properties(dac_ad5339_test PROPERTIES CXX_STANDARD_REQUIRED ON)
#set_target_properties(dac_ad5339_test PROPERTIES CXX_EXTENSIONS OFF)
#target_include_directories(dac_ad5339_test PRIVATE ${PROJECT_SOURCE_DIR}/dac)
#target_link_libraries(dac_ad5339_test i2c)

add_executable(test_signed_casting EXCLUDE_FROM_ALL test_signed_casting.cpp)
target_link_libraries(test_signed_casting PUBLIC common spdlog::spdlog)

#message(WARNING "source : ${PROJECT_SOURCE_DIR}/src/cib_daq_interface.cpp")
add_executable(test_json_config EXCLUDE_FROM_ALL test_json_config.cpp)
target_include_directories(test_json_config PUBLIC ${PROJECT_BINARY_DIR})
#target_link_libraries(test_json_config )

add_executable(test_struct_management EXCLUDE_FROM_ALL test_struct_management.cpp)
target_include_directories(test_struct_management PUBLIC ${PROJECT_SOURCE_DIR}/contrib)
target_include_directories(test_struct_management PUBLIC ${PROJECT_BINARY_DIR})

add_executable(test_namespace EXCLUDE_FROM_ALL test_namespace.cpp)
target_compile_features(test_namespace PUBLIC cxx_std_11)
target_include_directories(test_namespace PUBLIC ${PROJECT_SOURCE_DIR}/common)
target_include_directories(test_namespace PUBLIC ${PROJECT_BINARY_DIR})

