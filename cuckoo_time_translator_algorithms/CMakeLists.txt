cmake_minimum_required(VERSION 2.8.3)
project(cuckoo_time_translator_algorithms)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Eigen REQUIRED)
find_package(console_bridge REQUIRED)

include_directories(
  ${console_bridge_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# enable warnings
set (CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unknown-pragmas ${CMAKE_CXX_FLAGS}")

# enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.1")
  if (MSVC)
    message(FATAL_ERROR "CMake version 3.1 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
  endif ()
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
  else ()
    set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

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
  src/SlidingWindowOwt.cpp
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
