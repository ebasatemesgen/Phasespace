cmake_minimum_required(VERSION 2.8.3)
project(phasespace)

### Mechanism to tell whether we are compiling on the drone or not
### smaller version of https://github.com/axr/solar-cmake/blob/73cfea0db0284c5e2010aca23989046e5bda95c9/TargetArch.cmake
set(archdetect_c_code "
#if defined(__i386) || defined(__i386__) || defined(_M_IX86)
    #error cmake_ARCH i386
#elif defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
    #error cmake_ARCH x86_64
#elif defined(__arm__) || defined(__TARGET_ARCH_ARM)
    #error cmake_ARCH arm
#else
    #error cmake_ARCH other
#endif
")

function(is_x86 output_var)
  file(WRITE "${CMAKE_BINARY_DIR}/arch.c" "${archdetect_c_code}")
  enable_language(C)
  try_run(
      run_result_unused
      compile_result_unused
      "${CMAKE_BINARY_DIR}"
      "${CMAKE_BINARY_DIR}/arch.c"
      COMPILE_OUTPUT_VARIABLE ARCH
      CMAKE_FLAGS
  )


  # Parse the architecture name from the compiler output
  string(REGEX MATCH "cmake_ARCH ([a-zA-Z0-9_]+)" ARCH "${ARCH}")

  # Get rid of the value marker leaving just the architecture name
  string(REPLACE "cmake_ARCH " "" ARCH "${ARCH}")

  if (NOT ARCH)
    set(ARCH unknown)
  endif()

  set(${output_var} "${ARCH}" PARENT_SCOPE)
endfunction()
### End mechanism to tell if we are compiling on the drone

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp tf2 tf2_ros phasespace_msgs)
find_package(PkgConfig)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs roscpp phasespace_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

link_directories(${catkin_LIBRARY_DIRS} bin)

add_library(owlsock SHARED IMPORTED)
set_target_properties(owlsock PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/libowlsock.so)

# libowlsock.so is a shared object file compiled for x86 only. The drone runs ARM,
# so we cannot compile any executables that use libowlsock on the drone
is_x86(can_compile_owlsock)

if(can_compile_owlsock STREQUAL "i386" OR can_compile_owlsock STREQUAL "x86_64")
  add_executable(rigid_tracker src/rigid_tracker.cpp)
  add_dependencies(rigid_tracker ${catkin_EXPORTED_TARGETS})
  target_link_libraries(rigid_tracker owlsock ${catkin_LIBRARIES})
endif()
