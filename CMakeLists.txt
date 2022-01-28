cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 11)
project(vision VERSION 1.0)

file(GLOB VISION_SRC "src/*")


#link_directories(~/LIBS/opencv4/lib64)
#include_directories(~/LIBS/opencv4/include/opencv4)

add_executable(vision ${VISION_SRC})

target_include_directories(vision PRIVATE include)

#target_include_directories(vision PRIVATE /home/pi/LIBS-e/opencv40/include/opencv4)
#target_link_directories(vision PRIVATE /home/pi/LIBS-e/opencv40/lib)

target_include_directories(vision PRIVATE /usr/local/include/opencv4)
target_link_directories(vision PRIVATE /usr/local/lib/)

target_link_libraries(vision
opencv_core
opencv_ml
opencv_calib3d
opencv_flann
opencv_features2d
opencv_videoio
opencv_imgcodecs
opencv_highgui
opencv_imgproc
pthread)

