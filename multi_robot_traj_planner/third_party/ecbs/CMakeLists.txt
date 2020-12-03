cmake_minimum_required(VERSION 3.5)

project(ecbs)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        rospy
        std_msgs
        sensor_msgs)

# check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target( docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

# clang-tidy target (linter & static code analysis)
add_custom_target(clang-tidy
  COMMAND CMAKE_EXPORT_COMPILE_COMMANDS=ON run-clang-tidy-3.8.py ${CMAKE_CURRENT_SOURCE_DIR})

# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/a_star_epsilon.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ecbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/neighbor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/planresult.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a_star_epsilon.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ecbs.cpp
        ../../include/timer.hpp
)

add_custom_target(clang-format
  COMMAND clang-format -i ${ALL_SOURCE_FILES}
)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

include_directories(
  include
)

# Src

### a_star_epsilon
#add_executable(a_star_epsilon
#  src/a_star_epsilon.cpp
#)
#target_link_libraries(a_star_epsilon
#  ${Boost_LIBRARIES}
#)

### ecbs
#add_executable(ecbs
#  src/ecbs.cpp
#)
#target_link_libraries(ecbs
#  ${Boost_LIBRARIES}
#  yaml-cpp
#)
### a_star
#add_executable(a_star
#  src/a_star.cpp
#)
#target_link_libraries(a_star
#  ${Boost_LIBRARIES}
#)
#
### cbs
#add_executable(cbs
#  src/cbs.cpp
#)
#target_link_libraries(cbs
#  ${Boost_LIBRARIES}
#  yaml-cpp
#)