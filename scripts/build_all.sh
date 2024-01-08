#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"


pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    echo "Default ROS1"
    ROS_VERSION=${VERSION_ROS1}
    # exit
fi
echo "ROS version is: "$ROS_VERSION

for arg in "$@"; do
  if [[ $arg == "-j"* ]]; then
    # 使用正则表达式提取-j后面的数字
    number=$(echo "$arg" | sed 's/[^0-9]*//g')
    echo "compile thread: $number"
    NR_JOBS="$number"
    CATKIN_JOBS="-j${NR_JOBS}"
  else
    NR_JOBS=""
    CATKIN_JOBS=""
  fi
done


if [1]; then
    # clear `build/` folder.
    # TODO: Do not clear these folders, if the last build is based on the same ROS version.
    rm -rf ../../build/
    rm -rf ../../devel/
    rm -rf ../../install/
    # clear src/CMakeLists.txt if it exists.
    if [ -f ../CMakeLists.txt ]; then
        rm -f ../CMakeLists.txt
    fi
fi
# exit

# # substitute the files/folders: CMakeList.txt, package.xml(s)
# if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
#     if [ -f package.xml ]; then
#         rm package.xml
#     fi
#     cp -f package_ROS1.xml package.xml
# elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
#     if [ -f package.xml ]; then
#         rm package.xml
#     fi
#     cp -f package_ROS2.xml package.xml
#     cp -rf launch_ROS2/ launch/
# fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    # catkin_make -DROS_EDITION=${VERSION_ROS1}
    catkin build ${CATKIN_JOBS} -DROS_EDITION=${VERSION_ROS1}  livox_ros_driver livox_ros_driver2 
    catkin build ${CATKIN_JOBS} -DROS_EDITION=${VERSION_ROS1}  fast_lio
    #catkin build ${CATKIN_JOBS} -DROS_EDITION=${VERSION_ROS1}  rtabmap_ros
    catkin build ${CATKIN_JOBS} -DROS_EDITION=${VERSION_ROS1}  pc_process
    catkin build ${CATKIN_JOBS} -DROS_EDITION=${VERSION_ROS1}  a1_description unitree_controller unitree_gazebo unitree_legged_control
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd ../../
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null
