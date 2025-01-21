cmake_minimum_required(VERSION 2.8.3)
project(hmrcslam)

set(PERFORMANCE_MODE TRUE)

set(DEBUG_MODE 0)

IF(G2O_U14)
MESSAGE( STATUS "U14 Configuration")
add_definitions(-DU14_G2O)
ENDIF()

# This code was not tested with ROS version vefore "indigo"
if($ENV{ROS_DISTRO} MATCHES "indigo")
set(ROS_VERSION_INDIGO TRUE)
else()
set(ROS_VERSION_INDIGO FALSE)
endif()

set(CMAKE_CXX_FLAGS "-std=c++1y -pthread -pipe ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-reorder -Wno-sign-compare -Wno-unused-function -Wno-strict-aliasing ${CMAKE_CXX_FLAGS}")
if(NOT ${ROS_VERSION_INDIGO})
set(CMAKE_CXX_FLAGS "-Wno-unused-variable ${CMAKE_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "-Werror=parentheses -Werror=unused-value ${CMAKE_CXX_FLAGS}")
if(${ROS_VERSION_INDIGO})
set(CMAKE_CXX_FLAGS "-Werror=unused-variable ${CMAKE_CXX_FLAGS}")
endif()

if(${PERFORMANCE_MODE})
    set(CMAKE_CXX_FLAGS "-Wfatal-errors ${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif()

if(${ROS_VERSION_INDIGO})
    set(CMAKE_CXX_FLAGS "-L${PROJECT_SOURCE_DIR}/thirdparty/opencv-2.4.13/build/lib/ -lopencv_nonfree ${CMAKE_CXX_FLAGS}")
endif()

if(${DEBUG_MODE})
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
endif()

MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )

#ROS_BUILD_TYPE: Set the build type. Options are (default: RelWithDebInfo):
#Debug : w/ debug symbols, w/o optimization
#Release : w/o debug symbols, w/ optimization
#RelWithDebInfo : w/ debug symbols, w/ optimization
#RelWithAsserts : w/o debug symbols, w/ optimization, w/ assertions (i.e., w/o -DNDEBUG). New in ros 1.1.
#MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
if(${PERFORMANCE_MODE})
    set(CMAKE_BUILD_TYPE Release)
endif()

MESSAGE( STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE} )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(NOT ${ROS_VERSION_INDIGO})
    find_package(OpenCV REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge tf hmrcslam_msgs pcl_ros tf_conversions image_transport std_msgs message_generation)
find_package(cmake_modules REQUIRED)

add_service_files(
  FILES
  ServiceSaveMap.srv
)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

if(NOT ${ROS_VERSION_INDIGO})
find_package(Eigen3 REQUIRED 3.0)
else()
find_package(Eigen REQUIRED 3.0)
endif()
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
find_package(PCL 1.3 REQUIRED)

if(${ROS_VERSION_INDIGO})
    set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/thirdparty/opencv-2.4.13/build)
    set(OpenCV_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/opencv-2.4.13/build/include)
    find_package(OpenCV 2.4.13 REQUIRED)
endif()

set(THIRDPARTY_LIBS
  ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
  ${OpenCV_LIBS}
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS hmrcslam_msgs
   LIBRARIES hmrcslam_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/thirdparty
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(EXTERNAL_LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}
)

ADD_SUBDIRECTORY( include/hmrc_slam )

add_library(hmrcslam_lib
#cpp files
  src/config.cpp
  src/CentralControl.cpp
  src/ClientHandler.cpp
  src/Communicator.cpp
  src/Converter.cc
  src/Database.cpp
  src/Frame.cpp
  src/Initializer.cpp
  src/KeyFrame.cpp
  src/LoopFinder.cpp
  src/Map.cpp
  src/MapMatcher.cpp
  src/MapMerger.cpp
  src/Mapping.cpp
  src/MapPoint.cpp
  src/Optimizer.cpp
  src/ORBextractor.cpp
  src/ORBmatcher.cpp
  src/PnPSolver.cpp
  src/Sim3Solver.cpp
  src/Tracking.cpp
  src/Viewer.cpp

#header
  include/hmrc_slam/config.h
  include/hmrc_slam/CentralControl.h
  include/hmrc_slam/ClientHandler.h
  include/hmrc_slam/Communicator.h
  include/hmrc_slam/Converter.h
  include/hmrc_slam/Database.h
  include/hmrc_slam/Datatypes.h
  include/hmrc_slam/Frame.h
  include/hmrc_slam/Initializer.h
  include/hmrc_slam/KeyFrame.h
  include/hmrc_slam/LoopFinder.h
  include/hmrc_slam/Map.h
  include/hmrc_slam/MapMatcher.h
  include/hmrc_slam/MapMerger.h
  include/hmrc_slam/Mapping.h
  include/hmrc_slam/MapPoint.h
  include/hmrc_slam/Optimizer.h
  include/hmrc_slam/ORBextractor.h
  include/hmrc_slam/ORBmatcher.h
  include/hmrc_slam/ORBVocabulary.h
  include/hmrc_slam/PnPSolver.h
  include/hmrc_slam/Sim3Solver.h
  include/hmrc_slam/Tracking.h
  include/hmrc_slam/Viewer.h
  include/hmrc_slam/estd.h
)

add_dependencies(hmrcslam_lib hmrcslam_msgs_generate_messages_cpp)
target_link_libraries(hmrcslam_lib ${EXTERNAL_LIBRARIES} ${THIRDPARTY_LIBS})

add_executable(hmrcslamClientNode
#node
  src/client/ClientNode.cpp

#cpp files
  src/client/ClientSystem.cpp

#header
  include/hmrc_slam/client/ClientSystem.h
)
add_dependencies(hmrcslamClientNode hmrcslam_lib)
target_link_libraries(hmrcslamClientNode ${EXTERNAL_LIBRARIES} hmrcslam_lib)

add_executable(hmrcslamServerNode
#node
  src/server/ServerNode.cpp

#cpp files
  src/server/ServerSystem.cpp

#header
  include/hmrc_slam/server/ServerSystem.h
)
add_dependencies(hmrcslamServerNode hmrcslam_lib)
target_link_libraries(hmrcslamServerNode ${EXTERNAL_LIBRARIES} hmrcslam_lib)
