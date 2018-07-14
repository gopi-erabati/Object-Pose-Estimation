cmake_minimum_required(VERSION 2.8.9)
project(BuildModel)

#headers
include_directories(include)

#Sources
set ( SOURCES
 src/main.cpp
 src/datagrabber.cpp
 src/objectsegmentationplane.cpp
 src/processingpcd.cpp
 src/regmeshpcd.cpp
 src/rosinterface.cpp
 src/geometry.cpp
)

SET(CMAKE_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
				message_filters
				cv_bridge
				roslib
				roscpp
				sensor_msgs
				rosconsole
                                image_transport
				)

# Next lines needed for building all Qt projects
#find_package( Qt4 REQUIRED )
#include( ${QT_USE_FILE} )
#add_definitions( ${QT_DEFINITIONS} )
#include_directories( ${CMAKE_BINARY_DIR} )

# using Qt meta-system (precompiler)
#QT4_WRAP_CPP( MOC_SRCS ${MOC_HEADERS} )

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})

add_executable(BuildModel ${SOURCES} ${MOC_SRCS})

target_link_libraries(BuildModel ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
#${QT_LIBRARIES})

