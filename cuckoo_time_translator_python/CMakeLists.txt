cmake_minimum_required(VERSION 2.8.3)
project(cuckoo_time_translator_python)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_REQUIRED)

# Set up the python exports.

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


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

catkin_python_setup()

cs_add_library(${PROJECT_NAME}
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARY})

cs_install()
cs_export()

if(CATKIN_ENABLE_TESTING)
catkin_add_nosetests( 
  test/Test.py
  DEPENDENCIES ${PROJECT_NAME}
)
endif()
