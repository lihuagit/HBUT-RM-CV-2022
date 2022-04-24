find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

file(GLOB_RECURSE src *.cpp)

add_subdirectory(serial)

add_library(core_io OBJECT ${src})
target_link_libraries(core_io ${OpenCV_LIBS} Threads::Threads MVSDK serial)
