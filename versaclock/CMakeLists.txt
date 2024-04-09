

set(CMAKE_VERBOSE_MAKEFILE ON)
add_executable(program_vc program_vc.c)
target_include_directories(program_vc PRIVATE ${PROJECT_SOURCE_DIR}/versaclock)
