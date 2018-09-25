cmake_minimum_required(VERSION 2.8)
project(youbot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
)

add_subdirectory(src)

add_custom_target(copy_geometry ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/geometry ${CMAKE_BINARY_DIR}/geometry
)

#add_custom_target(copy_data ALL
#  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/data ${CMAKE_BINARY_DIR}/data)
