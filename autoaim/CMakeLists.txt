cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres 2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})

FILE(GLOB_RECURSE detector_src "./detector/*.cpp" "./detector/find/*cpp" "./detector/classifier/*.cpp")
aux_source_directory(predictor predictor_src)

add_library(autoaim OBJECT autoaim.cpp ${detector_src} ${predictor_src})
target_link_libraries(autoaim ${OpenCV_LIBS} ${CERES_LIBRARIES})
