# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(motorControlBasic)
find_package(YARP)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_library(libqpOASES NAMES libqpOASES.so PATHS "/home/michael/icub/software/src/robotology-superbuild/external/qpOASES/lib/")
set(qpOASES_DIR "/home/michael/icub/software/src/robotology-superbuild/external/qpOASES")
include_directories(${qpOASES_DIR}/include)


set(SDFAST 		"${CMAKE_CURRENT_SOURCE_DIR}/aux_and_IK/IKD/sdfast/")
set(CODES  		"${CMAKE_CURRENT_SOURCE_DIR}/aux_and_IK/IKD/")
set(ID 	   		"${CMAKE_CURRENT_SOURCE_DIR}/aux_and_IK/IKD/icub_id_cvxgen/")
set(IK 	   		"${CMAKE_CURRENT_SOURCE_DIR}/aux_and_IK/IKD/IKCG_cvxgen/")
set(WRAPPER 	"${CMAKE_CURRENT_SOURCE_DIR}/aux_and_IK/wrapper/")
set(STEPSGEN	"${CMAKE_CURRENT_SOURCE_DIR}/StepsGenerator/")
set(CODES_H		"${CMAKE_CURRENT_SOURCE_DIR}/include")
set(CODES_S		"${CMAKE_CURRENT_SOURCE_DIR}/src")


include_directories (${CODES}/include/)
include_directories (${SDFAST}/include/)
include_directories (${WRAPPER}/include/)
include_directories (${ID}/)
include_directories (${IK}/)
include_directories (${STEPSGEN}/include/)
include_directories (${CODES_H}/)


file(GLOB MODEL_SOURCES 	"${SDFAST}/*.c")
file(GLOB ID_SOURCES 		"${ID}/*.c")
file(GLOB IK_SOURCES 		"${IK}/*.c")
file(GLOB EXTRA_SOURCES 	"${CODES}/src/*.cpp")
file(GLOB WRAPPER_SOURCES 	"${WRAPPER}/src/*.cpp")
file(GLOB STEPSGEN_SOURCES 	"${STEPSGEN}/src/*.cpp")
file(GLOB CODES_SOURCES 	"${CODES_S}/*.cpp")
file(GLOB STACK_SOURCES 	"main.cpp")



add_definitions(-D_USE_MATH_DEFINES)
set(CMAKE_CXX_FLAGS "-std=c++11")	
# code optimization
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

add_library (Model  ${MODEL_SOURCES} ${ID_SOURCES} ${IK_SOURCES} ${EXTRA_SOURCES})
add_executable (main ${STACK_SOURCES} ${CODES_SOURCES} ${WRAPPER_SOURCES} ${STEPSGEN_SOURCES})
TARGET_LINK_LIBRARIES(main ${YARP_LIBRARIES} ${libqpOASES} Model)
