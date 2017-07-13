cmake_minimum_required(VERSION 2.8.3)
project(cuckoo_time_translator)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_REQUIRED)

# enable warnings
set (CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unknown-pragmas ${CMAKE_CXX_FLAGS}")

# enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.1")
  if (MSVC)
    message(FATAL_ERROR, "CMake version 3.1 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
  endif ()
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
  else ()
    set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

cs_add_library(${PROJECT_NAME}
  src/DeviceTimeTranslator.cpp
  src/RosConsoleBridgeActivator.cpp
)

cs_add_executable(mock_cuckoo_device_driver
  src/MockCuckooClock.cpp
  src/MockCuckooDeviceDriver.cpp
)
target_link_libraries(mock_cuckoo_device_driver ${PROJECT_NAME})

cs_install()
cs_export()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-unittest
    test/units.test
    test/TestDeviceTimeTranslator.cpp
    test/test_main.cpp
  )
  target_link_libraries(${PROJECT_NAME}-unittest ${PROJECT_NAME})
  
  add_rostest_gtest(${PROJECT_NAME}-nodetest
    test/device_time_translator.test
    test/TestDeviceTimeTranslatorNode.cpp
    test/test_main.cpp
  )
  target_link_libraries(${PROJECT_NAME}-nodetest ${PROJECT_NAME})
endif()
