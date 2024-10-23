

set(CMAKE_VERBOSE_MAKEFILE OFF)

#find_package(spdlog REQUIRED)
#find_package(i2c REQUIRED)


file(GLOB COMMON_SRC_FILES ${PROJECT_SOURCE_DIR}/common/*.cpp)
add_library(common SHARED ${COMMON_SRC_FILES})
target_compile_features(common PUBLIC cxx_std_11)
set_target_properties(common PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(common PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(common PUBLIC pthread )
