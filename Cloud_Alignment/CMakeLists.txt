cmake_minimum_required(VERSION 2.8)
project( cloud_align )
#find_package( OpenCV REQUIRED )
add_executable( cloud_align src/main.cpp src/gps_utils.cpp src/gps_utils.h)
#target_link_libraries( Grid ${OpenCV_LIBS} )

