CMAKE_MINIMUM_REQUIRED(VERSION 3.14.0)
PROJECT(Implement LANGUAGES CXX)

######################################################
###################### SETTINGS ######################
######################################################
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -fopenmp -DNDEBUG")
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_PREFIX_PATH ${CONDA_PREFIX} ${CMAKE_PREFIX_PATH})

ADD_DEFINITIONS(-DMKL_ILP64 -DEIGEN_USE_MKL_ALL) 
SET(MKL_ROOT "~/intel/oneapi/mkl/latest")
SET(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${MKL_ROOT}")
FIND_PACKAGE(MKL REQUIRED COMPONENTS
    mkl_intel_lp64  # 使用LP64接口（网页2强调必须替换ILP64）
    # mkl_sequential  # 单线程模式
    mkl_intel_thread  # 多线程库（原配置使用mkl_sequential为单线程）
    mkl_core
)


FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem thread serialization program_options)
FIND_PACKAGE(CGAL REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED PATHS ${CONDA_PREFIX}/include/eigen3)
FIND_PACKAGE(spdlog REQUIRED)
FIND_PACKAGE(OpenMP REQUIRED)
IF(OpenMP_FOUND)
    message("OpenMP version: ${OpenMP_CXX_VERSION}")
ENDIF()

ADD_SUBDIRECTORY(utility)

INCLUDE_DIRECTORIES(
    inc                 # 上层头文件路径
    ${SUB_INCLUDE_DIR}  # 下级头文件目录
)
INCLUDE_DIRECTORIES(BEFORE ${EIGEN3_INCLUDE_DIR})

AUX_SOURCE_DIRECTORY(.          SRC_LIST)
AUX_SOURCE_DIRECTORY(src        SRC_LIST)

ADD_EXECUTABLE(${PROJECT_NAME} ${SRC_LIST})
ADD_DEPENDENCIES(${PROJECT_NAME} Utility)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen  
    MKL::MKL
    iomp5
    OpenMP::OpenMP_CXX
    pthread dl m  
    Boost::filesystem Boost::system 
    ${CGAL_LIBRARIES}
    spdlog::spdlog
    Utility
)


