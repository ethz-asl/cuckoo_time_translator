cmake_minimum_required(VERSION 2.8.3)
project(cuckoo_time_translator_python)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_REQUIRED)

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


find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_python_setup()

cs_add_library(${PROJECT_NAME}
  src/module.cpp
)

string(REGEX REPLACE "_python$" "" LIB_OUTPUT_DIR ${CATKIN_PACKAGE_PYTHON_DESTINATION})
set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${LIB_OUTPUT_DIR}
)

if (APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".so" # This is apparently needed for Darvin. But why
  )
endif ()

target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARY} ${Boost_LIBRARIES})

install(FILES ${CATKIN_DEVEL_PREFIX}/${LIB_OUTPUT_DIR}/lib${PROJECT_NAME}.so
  DESTINATION ${LIB_OUTPUT_DIR}
)

cs_export()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    test/Test.py
    DEPENDENCIES ${PROJECT_NAME}
  )
endif()
