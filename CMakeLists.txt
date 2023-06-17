# 指定最低版本要求
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# 项目名称
project(MyProject)

# 查找 PCL 库
find_package(PCL REQUIRED)

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# 添加头文件路径
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/NDTMapCode 
        ${PCL_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

# 添加库路径
link_directories(
        include
        ${PCL_LIBRARY_DIRS}
        )

# 添加库 target
add_library(NDTMapCode STATIC
        NDTMapCode/NDTMC.cpp
        NDTMapCode/NDTMC.h
        )

# 链接 PCL 和 Eigen3 库到 NDTMapCode 库
target_link_libraries(NDTMapCode ${PCL_LIBRARIES} Eigen3::Eigen)

# 添加可执行文件 target
add_executable(run_demo
        run_demo.cpp
        )

# 链接 NDTMapCode 库到 run_demo 可执行文件
target_link_libraries(run_demo NDTMapCode ${PCL_LIBRARIES} -lpthread)