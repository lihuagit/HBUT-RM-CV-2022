cmake_minimum_required(VERSION 3.10)
project(CVRM2022)

set(CMAKE_CXX_STANDARD 17)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()

find_package(Threads REQUIRED)

include_directories(UltraMultiThread/include)
include_directories(others/include)

FILE(GLOB_RECURSE sourcefiles "others/src/*.cpp")

add_executable(CVRM2022 main.cpp ${sourcefiles})
target_link_libraries(CVRM2022)

include_directories(core_io)
add_subdirectory(core_io)
target_link_libraries(CVRM2022 core_io)

include_directories(autoaim)
add_subdirectory(autoaim)
target_link_libraries(CVRM2022 autoaim)

include_directories(img_show)
add_subdirectory(img_show)
target_link_libraries(CVRM2022 img_show)
