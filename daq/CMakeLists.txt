

set(CMAKE_VERBOSE_MAKEFILE ON)

#find_package(spdlog REQUIRED)
#find_package(i2c REQUIRED)


add_library(cib_daq SHARED Handler.cpp ReaderBase.cpp ReaderAXIFIFO.cpp)
target_include_directories(cib_daq PUBLIC ${PROJECT_SOURCE_DIR}/contrib/json/include/nlohmann)
target_include_directories(cib_daq PUBLIC ${PROJECT_SOURCE_DIR}/contrib/json/include)
target_include_directories(cib_daq PUBLIC ${PROJECT_SOURCE_DIR}/common)
target_include_directories(cib_daq PUBLIC ${PROJECT_SOURCE_DIR}/contrib)
target_include_directories(cib_daq PUBLIC ${PROJECT_SOURCE_DIR}/daq)
target_link_libraries(cib_daq PUBLIC ${Boost_LIBRARIES} spdlog )
