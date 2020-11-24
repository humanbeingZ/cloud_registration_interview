project(cloud_registration)
cmake_minimum_required(VERSION 3.0.0)

#########################################
find_package(PCL REQUIRED)

#########################################
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

#########################################
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#########################################
include_directories(include)
link_directories(src)

#########################################
add_executable(cloud_registration app/cloud_registration.cpp src/icp.cpp src/visualization.cpp)
target_link_libraries(cloud_registration ${PCL_LIBRARIES})

