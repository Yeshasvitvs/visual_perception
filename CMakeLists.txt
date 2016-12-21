cmake_minimum_required(VERSION 2.6)
project(visual_perception)
#message(STATUS "PROJECT_SOURCE_DIR : " ${PROJECT_SOURCE_DIR})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(OpenCV_FOUND TRUE)
set(OpenCV_DIR /usr/local/share/OpenCV)
#set(OpenCV_INCLUDE_DIRS /usr/local/include/opencv)

message(STATUS "OpenCV_DIR : "${OpenCV_DIR})
find_package(OpenCV REQUIRED)
message(STATU "OpenCV Version : " ${CV_MAJOR_VERSION})
message(STATUS "OpenCV_LIBS : " ${OpenCV_LIBS})
message(STATUS "OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})

find_package(YARP REQUIRED)
message(STATUS "YARP_INCLUDE_DIRS : " ${YARP_INCLUDE_DIRS})
message(STATUS "YARP_LIBRARIES : " ${YARP_LIBRARIES} )

find_package(ICUB REQUIRED)
message(STATUS "ICUB_INCLUDE_DIRS : " ${ICUB_INCLUDE_DIRS})
message(STATUS "ICUB_LIBRARIES : " ${ICUB_LIBRARIES})

find_package(Boost REQUIRED)
message(STATUS "BOOST_INCLUDE_DIRS : " ${Boost_INCLUDE_DIRS})
message(STATUS "BOOST_LIBRARIES : " ${Boost_LIBRARIES})

#set(SOURCE_FILES main.cpp frame_grabber.cpp)
#set(HEADER_FILES utils.h frame_grabber.h)

#message(STATUS "SOURCE_FILES : " ${SOURCE_FILES})
#message(STATUS "HEADER_FILES : " ${HEADER_FILES})

include_directories(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} -I include/)

add_executable(visual_perception src/main.cpp src/visual_module.cpp src/frame_grabber.cpp src/pose_estimate.cpp)

target_link_libraries(visual_perception ${OpenCV_LIBS} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Boost_LIBRARIES} -lm -ldl)

#install(TARGETS visual_perception RUNTIME DESTINATION bin)
