
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(read_axi_fifo read_axi_fifo.cpp)
#target_include_directories(check_timing ${PROJECT_SOURCE_DIR}/common)
target_compile_features(read_axi_fifo PUBLIC cxx_std_11)
set_target_properties(read_axi_fifo PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(read_axi_fifo PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(read_axi_fifo PRIVATE ${PROJECT_SOURCE_DIR}/axi_fifo)
target_link_libraries(read_axi_fifo pthread)

add_executable(read_daq_fifo read_daq_fifo.cpp)
target_compile_features(read_daq_fifo PUBLIC cxx_std_11)
set_target_properties(read_daq_fifo PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(read_daq_fifo PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(read_daq_fifo PRIVATE ${PROJECT_SOURCE_DIR}/axi_fifo)
target_link_libraries(read_daq_fifo pthread common)
