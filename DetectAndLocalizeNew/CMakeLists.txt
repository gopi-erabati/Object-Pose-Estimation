cmake_minimum_required(VERSION 2.8.9)
project(DetectAndLocalize)

#headers
include_directories(include)

#packages
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

#sources
set ( SOURCES
 src/main.cpp
 src/datagrabber.cpp
 src/objectsegmentationplane.cpp
 src/poseestimator.cpp
 src/processingpcd.cpp
 src/rosinterface.cpp
 src/geometry.cpp
# src/miscellaneous.cpp #To Include this module, please include QT Projects in this same CMakeLists
# src/extractsurf.cpp
# src/kinectgrabber.cpp
# src/regmeshpcd.cpp
# src/segmentationregiongrow.cpp
)

if(${PCL_VERSION} VERSION_GREATER "1.7.1")
set(SOURCES
    ${SOURCES}
    include/pcl/registration/impl/correspondence_rejection_self_occluded_normal.cpp
    )
endif()

#build Type Release or Debug
SET(CMAKE_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Next lines needed for building all Qt projects
#find_package( Qt4 REQUIRED )
#include( ${QT_USE_FILE} )
#add_definitions( ${QT_DEFINITIONS} )
#include_directories( ${CMAKE_BINARY_DIR} )

# using Qt meta-system (precompiler)
#QT4_WRAP_CPP( MOC_SRCS ${MOC_HEADERS} )

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})

add_executable(DetectAndLocalize ${SOURCES} ${MOC_SRCS})

target_link_libraries(DetectAndLocalize ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})# ${QT_LIBRARIES})
