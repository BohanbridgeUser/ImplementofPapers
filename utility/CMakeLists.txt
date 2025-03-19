CMAKE_MINIMUM_REQUIRED( VERSION 3.14.0 )
PROJECT(Utility LANGUAGES CXX)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -fopenmp -DNDEBUG")
######################################################
###################### SETTINGS ######################
######################################################
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

######################################################
###################### EXTERNAL ######################
######################################################

######################################################
###################### INTERNAL ######################
######################################################
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SUB_INCLUDE_DIR 
    ${SUB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include                 
    CACHE INTERNAL "preprocess include dir"   
)

INCLUDE_DIRECTORIES(
    ./include
)

SET(SRC_LIST
    ./src/my_thread_pool.cpp
)
# AUX_SOURCE_DIRECTORY(src        SRC_LIST)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${SRC_LIST})