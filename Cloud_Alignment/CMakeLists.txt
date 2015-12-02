cmake_minimum_required(VERSION 2.8)
project(cloud_align)
find_package(OpenCV 2.4 REQUIRED)

find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig)

pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions ( ${PCL_DEFINITIONS} )


set (CMAKE_CXX_FLAGS "-g")

file(GLOB SFM_LIB
	src/*
)

add_library(cloud_align_LIB ${SFM_LIB})
add_executable (cloud_align main.cpp)
target_link_libraries(cloud_align  
			cloud_align_LIB
			${PCL_LIBRARIES}
					
)

