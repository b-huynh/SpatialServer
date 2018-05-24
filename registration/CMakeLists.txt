cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(hsm-registration)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pairwise_registration pairwise_registration.cpp)
target_link_libraries (pairwise_registration ${PCL_LIBRARIES})

add_executable(interactive_icp interactive_icp.cpp)
target_link_libraries(interactive_icp ${PCL_LIBRARIES})
