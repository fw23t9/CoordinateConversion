cmake_minimum_required(VERSION 3.10)

set(${PROJECT_NAME} "conversionDemo")

project (conversionDemo VERSION 1.0)

set (CMAKE_DEBUG_POSTFIX "d")

file(GLOB SOURCE_FILES ./src/*.*)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

#### Eigen ####
# find_package(Eigen3 REQUIRED)

#### add your own eigen_dir here ####
set (Eigen_Dir "D:/Program Files/Eigen3/include/eigen3/")

include_directories(${Eigen_Dir})
