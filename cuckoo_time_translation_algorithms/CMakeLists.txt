cmake_minimum_required(VERSION 2.8.3)
project(cuckoo_time_translation_algorithms)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Eigen REQUIRED)
find_package(console_bridge REQUIRED)

include_directories(
  ${console_bridge_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(--std=c++11)

if(roscpp_FOUND)
    MESSAGE(STATUS "Found ROS. Going to use its assertions")
    add_definitions(-DUSE_ROS_ASSERTIONS)
endif()


cs_add_library(${PROJECT_NAME}
  src/OneWayTranslator.cpp
  src/ConvexHullOwt.cpp
  src/TimestampUnwrapper.cpp
  src/KalmanOwt.cpp
  src/SwitchingOwt.cpp
)
target_link_libraries(${PROJECT_NAME} ${console_bridge_LIBRARIES})

cs_install()
cs_export(INCLUDE_DIRS ${Eigen_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/TestTimestampUnwrapper.cpp
    test/TestOneWayTranslators.cpp
    test/test_main.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
