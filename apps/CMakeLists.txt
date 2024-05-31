
#set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(cib_manager cib_manager.cpp)
target_compile_features(cib_manager PUBLIC cxx_std_11)
set_target_properties(cib_manager PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(cib_manager PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(cib_manager PRIVATE ${PROJECT_SOURCE_DIR}/cib_debug)
target_include_directories(cib_manager PRIVATE ${PROJECT_SOURCE_DIR}/common)
target_include_directories(cib_manager PUBLIC ${ZeroMQ_INCLUDE_DIR})
target_include_directories(cib_manager PRIVATE ${PROJECT_SOURCE_DIR}/i2c)

target_link_libraries(cib_manager cib_i2c common readline)
