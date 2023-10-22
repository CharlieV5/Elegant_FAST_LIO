cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

## 设置编译选项
SET(CMAKE_BUILD_TYPE "Debug")
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

## 标记Cmake根路径
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

## 获取可用线程数
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

## 尝试寻找OS中的OMP
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

## python依赖
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

## ROS依赖
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

## 其它依赖
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)

message(STATUS "## Is yaml-cpp found   : " yaml-cpp_FOUND)
message(STATUS "## YAML_CPP_INCLUDE_DIR: " ${YAML_CPP_INCLUDE_DIR})
message(STATUS "## YAML_CPP_LIBRARIES  : " ${YAML_CPP_LIBRARIES})

## 头文件搜索路径
include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  include
  src
)

# ## 自定义ros消息
# add_message_files(
#   FILES
#   Pose6D.msg
# )

# generate_messages(
#  DEPENDENCIES
#  geometry_msgs
# )

## 声明ros包依赖
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

## 编译目标 [ ********* Old Version ********* ]
add_executable(fastlio_mapping 
  src/laser_mapping.cpp 
  src/lidar_preprocess.cpp
  src/ikd-Tree/ikd_Tree.cpp 
  src/common/options.cpp
  src/core/imu_process.cpp
  src/io/yaml_io.cpp
  src/fast_lio.cpp
)
target_link_libraries(fastlio_mapping 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(fastlio_mapping PRIVATE 
  ${PYTHON_INCLUDE_DIRS}
)

## 编译目标 [ ********* New Version ********* ]
add_executable(run_fast_lio_node
  src/run_fast_lio_node_main.cpp 
  src/lidar_preprocess.cpp
  src/ikd-Tree/ikd_Tree.cpp 
  src/common/options.cpp
  src/core/imu_process.cpp
  src/io/yaml_io.cpp
  src/fast_lio.cpp
)
target_link_libraries(run_fast_lio_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(run_fast_lio_node PRIVATE 
  ${PYTHON_INCLUDE_DIRS}
)