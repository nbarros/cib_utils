
set(CMAKE_VERBOSE_MAKEFILE ON)

#find_package(cppzmq)
## load in pkg-config support
find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)
## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from above to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )


add_executable(test_lbls test_lbls_interface.cpp)
target_compile_features(test_lbls PUBLIC cxx_std_11)
set_target_properties(test_lbls PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(test_lbls PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(test_lbls PRIVATE ${PROJECT_SOURCE_DIR}/cib_debug)
target_include_directories(test_lbls PRIVATE ${PROJECT_SOURCE_DIR}/common)
target_include_directories(test_lbls PUBLIC ${ZeroMQ_INCLUDE_DIR})
target_link_libraries(test_lbls ${ZeroMQ_LIBRARY} pthread common)

add_executable(cib_manager cib_manager.cpp)
target_compile_features(cib_manager PUBLIC cxx_std_11)
set_target_properties(cib_manager PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(cib_manager PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(cib_manager PRIVATE ${PROJECT_SOURCE_DIR}/cib_debug)
target_include_directories(cib_manager PRIVATE ${PROJECT_SOURCE_DIR}/common)
target_include_directories(cib_manager PUBLIC ${ZeroMQ_INCLUDE_DIR})
target_link_libraries(cib_manager common readline)
