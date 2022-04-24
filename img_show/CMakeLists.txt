find_package(OpenCV 4 REQUIRED)

file(GLOB_RECURSE src *.cpp)

add_library(img_show OBJECT ${src})
target_link_libraries(img_show ${OpenCV_LIBS})