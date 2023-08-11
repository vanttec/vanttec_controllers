cmake_minimum_required(VERSION 3.5)

project(vanttec_controllers)


add_library(utils utils.cpp)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


# # Set the output folder where your program will be created
# set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
# set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# # The following folder will be included
# include_directories("${PROJECT_SOURCE_DIR}")
