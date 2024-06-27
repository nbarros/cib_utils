
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

set(Boost_DEBUG ON)
find_package(Boost COMPONENTS ALL)


add_executable(cib_daq_server cib_daq_server.cpp)
target_compile_features(cib_daq_server PUBLIC cxx_std_11)
target_compile_definitions(cib_daq_server PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
set_target_properties(cib_daq_server PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(cib_daq_server PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(cib_daq_server PRIVATE ${PROJECT_SOURCE_DIR}/common)
target_link_libraries(cib_daq_server cib_daq spdlog  ${Boost_LIBRARIES} pthread common)

#target_include_directories(cib_daq_interface PRIVATE ${PROJECT_SOURCE_DIR}/cib_debug)
#target_include_directories(cib_daq_interface PUBLIC ${ZeroMQ_INCLUDE_DIR})
#target_include_directories(cib_daq_interface PRIVATE ${PROJECT_SOURCE_DIR}/i2c)

