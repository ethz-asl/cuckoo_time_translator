cmake_minimum_required(VERSION 2.8.3)
project(cuckoo_time_translator)

find_package(catkin_simple REQUIRED)

catkin_simple( ALL_REQUIRED)

add_definitions(--std=c++11 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
  src/DeviceTimeTranslator.cpp
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
