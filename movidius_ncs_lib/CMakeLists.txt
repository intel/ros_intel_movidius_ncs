# Copyright (c) 2017 Intel Corporation
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(movidius_ncs_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rostest
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslint
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add opencv dependency
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_path(MVNC_HEADER_FILE
          mvnc.h
          PATHS ENV ROS_ROOT)
if(NOT MVNC_HEADER_FILE)
  message(FATAL_ERROR "Can't find mvnc.h, please install Movidius NCS SDK according to
          https://developer.movidius.com/start/software-setup")
endif()

add_library(${PROJECT_NAME}
  src/ncs.cpp
  src/device.cpp
  src/exception.cpp
  src/exception_util.cpp
  src/graph.cpp
  src/tensor.cpp
  src/result.cpp
  src/ncs_manager.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mvnc
)

# Flags
if(UNIX OR APPLE)
  # Linker flags.
  if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if( ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel" )
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

 # Add OpenMP support
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

# Add x86 intrinsic compiler support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
execute_process(
    COMMAND bash -c "lscpu | grep -qi flags | grep -qi flags | grep -qi f16c"
    RESULT_VARIABLE SUPPORT_F16C)
if (SUPPORT_F16C EQUAL 0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mf16c")
  add_definitions(-DSUPPORT_MF16C)
endif()

execute_process(
    COMMAND bash -c "lscpu | grep -qi flags | grep -qi flags | grep -qi sse4_1"
    RESULT_VARIABLE SUPPORT_SSE41)
if (SUPPORT_SSE41 EQUAL 0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
endif()

# Install nodelet library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(catkin REQUIRED COMPONENTS roslib)
  roslint_cpp()
  roslint_add_test()

  # rostest NCS SDK environment
  add_executable(unittest_environment tests/unittest_environment.cpp)
  target_link_libraries(unittest_environment ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${UNITEST_LIBRARIES})
  # rostest library function
  add_executable(unittest_function tests/unittest_function.cpp)
  target_link_libraries(unittest_function ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${UNITEST_LIBRARIES})
  add_dependencies(unittest_function ${catkin_EXPORTED_TARGETS})

  add_rostest(tests/test_all.test)
endif()
