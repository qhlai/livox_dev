cmake_minimum_required(VERSION 2.8.3)
project(pc_process)

SET(CMAKE_BUILD_TYPE "Debug")
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
if("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
    message(STATUS "building for: debug")
    add_definitions("-g")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -O1 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
else()
    message(STATUS "building for: release")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
endif()


add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
# include(FetchContent)
# FetchContent_Declare(dbg_macro GIT_REPOSITORY https://github.com/sharkdp/dbg-macro)
# FetchContent_MakeAvailable(dbg_macro)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

# Add OpenCV if available
# set(OpenCV_DIR "/home/uestc/opt/opencv/build")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZLIB)
find_package(GTSAM REQUIRED QUIET)
# find_package(cpprestsdk REQUIRED) #sudo apt-get install libcpprest-dev
find_package(Ceres REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/libs/dbg-macro)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(PCL: ${PCL_INCLUDE_DIRS})
message(OpenCV: ${OpenCV_INCLUDE_DIRS})
message(ZLIB: ${ZLIB_INCLUDE_DIRS})
message(GTSAM: ${GTSAM_INCLUDE_DIR})
message(Ceres: ${CERES_INCLUDE_DIRS})

set (INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/src
    # ${PROJECT_SOURCE_DIR}/lib/dbg-macro
    # ${PROJECT_SOURCE_DIR}/src/core
    # ${PROJECT_SOURCE_DIR}/src/map
    # ${PROJECT_SOURCE_DIR}/src/map/frames
    # ${PROJECT_SOURCE_DIR}/src/tools
    # ${PROJECT_SOURCE_DIR}/src/communicator
    # ${PROJECT_SOURCE_DIR}/src/communicator/server
    # ${PROJECT_SOURCE_DIR}/src/communicator/client
    # # ${PROJECT_SOURCE_DIR}/src/communicator/msgs
    # # ${PROJECT_SOURCE_DIR}/src/communicator/client/mersys_backend
    # ${PROJECT_SOURCE_DIR}/src/place_rec
    # ${PROJECT_SOURCE_DIR}/src/place_rec/calib
    # ${PROJECT_SOURCE_DIR}/src/visualizer
)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  # include
  ${INCLUDE_DIRS}
  # src/lib/dbg-macro
  ${PROJECT_SOURCE_DIR}/libs/dbg-macro
  )




catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)


file(GLOB_RECURSE POINTCLOUD_SRC_FILES   
${PROJECT_SOURCE_DIR}/src/pointcloud/*.cpp

)

file(GLOB_RECURSE POINTCLOUD_INC_FILES   
${PROJECT_SOURCE_DIR}/src/pointcloud/*.hpp

)

add_executable(pcd2ply src/transfer/pcd2ply.cpp)
target_link_libraries(pcd2ply ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pcd2ply PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(ply2pcd src/transfer/ply2pcd.cpp)
target_link_libraries(ply2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(ply2pcd PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(pcd2oct src/transfer/pcd2oct.cpp)
target_link_libraries(pcd2oct ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pcd2oct PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(ply2oct src/transfer/ply2oct.cpp)
target_link_libraries(ply2oct ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(ply2oct PRIVATE ${PYTHON_INCLUDE_DIRS})

# add_executable(ply2oct_vis src/transfer/ply2oct_vis.cpp)
# target_link_libraries(ply2oct_vis ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
# target_include_directories(ply2oct_vis PRIVATE ${PYTHON_INCLUDE_DIRS})



add_executable(rtabmap_test 
  src/rtabmap_test.cpp
  ${POINTCLOUD_SRC_FILES}
)
target_link_libraries(rtabmap_test
 ${catkin_LIBRARIES} 
 ${PCL_LIBRARIES} 
 ${PYTHON_LIBRARIES}
#  dbg_macro
 )
target_include_directories(rtabmap_test PRIVATE ${PYTHON_INCLUDE_DIRS})



add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(test PRIVATE 
${PYTHON_INCLUDE_DIRS} 
${PROJECT_SOURCE_DIR}/src/libs/dbg-macro
)