cmake_minimum_required (VERSION 3.8)

project("controller" VERSION 0.0.1
  DESCRIPTION "C++ implementation of PID controller"
  HOMEPAGE_URL "https://github.com/borodziejciesla/control")

set(TEST_PROJECT_NAME ${PROJECT_NAME}_tests)
set(EXAMPLE_PROJECT_NAME ${PROJECT_NAME}_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLE "Build example" ON)

###############################################
################## Library ####################
###############################################
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED QUIET)

add_library(${PROJECT_NAME}
    src/pid.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

###############################################
################# Examples ####################
###############################################
if(BUILD_EXAMPLE)
    project(${EXAMPLE_PROJECT_NAME})
    
    add_subdirectory(example)
endif() # BUILD_EXAMPLE

###############################################
################ Unit Tests ###################
###############################################
if(BUILD_TESTS)    
    project(${TEST_PROJECT_NAME})

    add_subdirectory(tests)    
    enable_testing()
    add_test(${TEST_PROJECT_NAME} ${TEST_PROJECT_NAME})    
endif() #   BUILD_TESTS