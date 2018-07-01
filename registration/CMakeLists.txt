cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(hsm-registration)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(interactive_icp interactive_icp.cpp)
target_link_libraries(interactive_icp ${PCL_LIBRARIES})

add_executable(sic_test sic_test.cpp)
target_link_libraries(sic_test ${PCL_LIBRARIES})

add_executable(sic_test_normal sic_test_normal.cpp)
target_link_libraries(sic_test_normal ${PCL_LIBRARIES})

add_executable(sic_test_general sic_test_general.cpp)
target_link_libraries(sic_test_general ${PCL_LIBRARIES})

add_executable(sic_test_crop_box sic_test_crop_box.cpp)
target_link_libraries(sic_test_crop_box ${PCL_LIBRARIES})
