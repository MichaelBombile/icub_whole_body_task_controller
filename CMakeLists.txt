
### ########################################################################################################################################
cmake_minimum_required(VERSION 3.5)
project(hwbtc_v61)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)

# set (CMAKE_CXX_STANDARD 17)

find_library(libqpOASES NAMES libqpOASES.so PATHS "${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES/lib/")
set(qpOASES_DIR "${ROBOTOLOGY_SUPERBUILD_ROOT}/external/qpOASES")
include_directories(${qpOASES_DIR}/include)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${YARP_INCLUDE_DIRS}) 
include_directories(${ICUB_INCLUDE_DIRS})

set(HEADERS_FOLDER 	"${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SRC_FOLDER 		"${CMAKE_CURRENT_SOURCE_DIR}/src")



set(ROBOT 	   		"${CMAKE_CURRENT_SOURCE_DIR}/Robot/")

set(IK1 	   		"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/wbIK1_cvxgen/")
set(IK2 	   		"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/wbIK2_cvxgen/")
set(IK3 	   		"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/wbIK3_cvxgen/")
set(IK4 	   		"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/wbIK4_cvxgen/")
set(CoMclp 	   		"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/CoMclp_cvxgen/")
set(legIK 	   		"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/legIK_cvxgen/")

# set(wbk 	   			"${CMAKE_CURRENT_SOURCE_DIR}/wbQPSolvers/wbk_cvxgen/")
set(SOLVER_ID 		"${CMAKE_CURRENT_SOURCE_DIR}/wbQPSolvers/cvxgen_solver_wb")
set(SOLVER_ID2 		"${CMAKE_CURRENT_SOURCE_DIR}/wbQPSolvers/wb2_cvxgen")   # Hierarchy
set(SOLVER_BWC 		"${CMAKE_CURRENT_SOURCE_DIR}/wbQPSolvers/bwc_cvxgen")
set(SOLVER_CMO 		"${CMAKE_CURRENT_SOURCE_DIR}/wbQPSolvers/cmo_cvxgen")
set(SOLVER_WBIK1 	"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/wbIK_cvxgen/")
set(SOLVER_CoP 	   	"${CMAKE_CURRENT_SOURCE_DIR}/wbQPSolvers/cop_cvxgen/")



set(STEPSGEN		"${CMAKE_CURRENT_SOURCE_DIR}/auxiliary/StepsGenerator/")
set(CODES_H			"${CMAKE_CURRENT_SOURCE_DIR}/auxiliary/include")
set(CODES_S			"${CMAKE_CURRENT_SOURCE_DIR}/auxiliary/src")
# ### Feedforward controller
set(ANTICIP_CTRL_H 	"${CMAKE_CURRENT_SOURCE_DIR}/AnticipatoryController/include")
set(ANTICIP_CTRL_S 	"${CMAKE_CURRENT_SOURCE_DIR}/AnticipatoryController/src")

set(BIMANUAL_CTRL_H	"${CMAKE_CURRENT_SOURCE_DIR}/BimanualControllers/include")
set(BIMANUAL_CTRL_S	"${CMAKE_CURRENT_SOURCE_DIR}/BimanualControllers/src")
set(IMU	   			"${CMAKE_CURRENT_SOURCE_DIR}/auxiliary/IMU")

set(WBID_H 			"${CMAKE_CURRENT_SOURCE_DIR}/wbID/include")
set(WBID_S 			"${CMAKE_CURRENT_SOURCE_DIR}/wbID/src")

set(WBIK_H 			"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/include")
set(WBIK_S 			"${CMAKE_CURRENT_SOURCE_DIR}/wbIK/src")


set(HEADERS    	${HEADERS_FOLDER}/WholeBodyTasksControlModule.h
				${HEADERS_FOLDER}/wbhpidcUtilityFunctions.hpp
			   	${HEADERS_FOLDER}/ControllerParameters.h
			   	${HEADERS_FOLDER}/Quaternions.h
			   	${HEADERS_FOLDER}/TasksReferences.h
			   	${HEADERS_FOLDER}/BalanceReference.h
               	${HEADERS_FOLDER}/Object_to_Grasp.hpp
               	${HEADERS_FOLDER}/WBData_logging.h
			   	)

set(SOURCES     ${SRC_FOLDER}/WholeBodyTasksControlModule.cpp
				${SRC_FOLDER}/ControllerParameters.cpp
				${SRC_FOLDER}/Quaternions.cpp
				${SRC_FOLDER}/TasksReferences.cpp
				${SRC_FOLDER}/BalanceReference.cpp
				${SRC_FOLDER}/WBData_logging.cpp
			   	main.cpp
			   	)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})


include_directories(${HEADERS_FOLDER}/)

# # -----------------------------------------------------------------------
# # robot interface
# # -----------------------------------------------------------------------
include_directories(${ROBOT}/include/)
include_directories (${IMU}/include/)
file(GLOB ROBOT_SOURCES		"${ROBOT}/src/*.cpp")
file(GLOB IMU_SOURCES 		"${IMU}/src/*.c"  
							"${IMU}/src/*.cpp")
# create a library
add_library (interface 			${ROBOT_SOURCES} ${IMU_SOURCES})
target_link_libraries(interface ${iDynTree_LIBRARIES} ${YARP_LIBRARIES})
# # -----------------------------------------------------------------------
# # QP solvers
# # -----------------------------------------------------------------------
# include_directories(${SOLVER_ID}/)
# include_directories(${SOLVER_ID2}/)
# include_directories(${SOLVER_BWC}/)
# include_directories(${SOLVER_CMO}/)
# include_directories(${SOLVER_WBIK1}/)

# file(GLOB ID_SOURCES 		"${SOLVER_ID}/*.c")
# file(GLOB ID2_SOURCES		"${SOLVER_ID2}/*.c")
# file(GLOB BWC_SOURCES		"${SOLVER_BWC}/*.c")
# file(GLOB CMO_SOURCES		"${SOLVER_CMO}/*.c")
# file(GLOB WBIK_SOL_SRC 		"${SOLVER_WBIK1}/*.c")

# add_library (solvers  	${ID_SOURCES} 
# 						${ID2_SOURCES} 
# 						${BWC_SOURCES} 
# 						${CMO_SOURCES} 
# 						${WBIK_SOL_SRC} 
# 									)

include_directories(${SOLVER_ID}/)
include_directories(${SOLVER_ID2}/)
include_directories(${IK1}/)
include_directories(${IK2}/)
include_directories(${IK3}/)
include_directories(${IK4}/)
include_directories(${legIK}/)
include_directories(${CoMclp})
include_directories(${SOLVER_BWC}/)
include_directories(${SOLVER_CMO}/)
include_directories(${SOLVER_WBIK1}/) #
include_directories(${SOLVER_CoP}/) #SOLVER_CoP

# include_directories(${wbk}/)

file(GLOB ID_SOURCES 		"${SOLVER_ID}/*.c")
file(GLOB ID2_SOURCES		"${SOLVER_ID2}/*.c")
file(GLOB IK_SOURCES1 		"${IK1}/*.c")
file(GLOB IK_SOURCES2 		"${IK2}/*.c")
file(GLOB IK_SOURCES3 		"${IK3}/*.c")
file(GLOB IK_SOURCES4 		"${IK4}/*.c")
file(GLOB IK_SOURCES_leg	"${legIK}/*.c")
file(GLOB CoMclp_SRC 		"${CoMclp}/*.c")
file(GLOB BWC_SOURCES		"${SOLVER_BWC}/*.c")
file(GLOB CMO_SOURCES		"${SOLVER_CMO}/*.c")
file(GLOB WBIK_SOL_SRC 		"${SOLVER_WBIK1}/*.c")
file(GLOB COP_SOL_SRC 		"${SOLVER_CoP}/*.c")
# file(GLOB wbk_SOURCES		"${wbk}/*.c")


# create a library
add_library (solvers  	${ID_SOURCES}
						${ID2_SOURCES}
						${IK_SOURCES1} 
						${IK_SOURCES2} 
						${IK_SOURCES3} 
						${IK_SOURCES4} 
						${CoMclp_SRC} 
						${BWC_SOURCES} 
						${CMO_SOURCES}
						${WBIK_SOL_SRC}
						${IK_SOURCES_leg}
						${COP_SOL_SRC}
# 						${wbk_SOURCES}
						)
# # -----------------------------------------------------------------------
# # controllers
include_directories(${STEPSGEN}/include/)
# include_directories(${CODES_wbIK_H}/)
include_directories(${CODES_H}/)
include_directories(${BIMANUAL_CTRL_H}/)
include_directories(${ANTICIP_CTRL_H}/)
include_directories(${WBID_H}/)
include_directories(${WBIK_H}/)
# include_directories (${IMU}/include/)
# #
file(GLOB WBIK_SOURCES 		"${CODES_wbIK_S}/*.cpp")
file(GLOB STEPSGEN_SOURCES 	"${STEPSGEN}/src/*.cpp")
file(GLOB CODES_SOURCES 	"${CODES_S}/*.cpp")
file(GLOB BIMAN_SOURCES 	"${BIMANUAL_CTRL_S}/*.cpp")
file(GLOB ANTICIP_SOURCES 	"${ANTICIP_CTRL_S}/*.cpp")


file(GLOB WBID_SOURCES 		"${WBID_S}/*.cpp")
file(GLOB WBIK_SOURCES 		"${WBIK_S}/*.cpp")
# file(GLOB IMU_SOURCES 		"${IMU}/src/*.c"  "${IMU}/src/*.cpp")

# #


# # -----------------------------------------------------------------------
# # Executable for the project
# # -----------------------------------------------------------------------
# add_executable(${PROJECT_NAME} 	
# 								${SOURCES} 
# # 								${ROBOT_SOURCES}  
# 								${CODES_SOURCES}
# 								${STEPSGEN_SOURCES}
# 								${BIMAN_SOURCES}
# 								${ANTICIP_SOURCES}
# 								${WBIK_SOURCES}
# 								${IMU_SOURCES}
# 								)
# # link to target library
# target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})

# target_link_libraries(${PROJECT_NAME} 	${iDynTree_LIBRARIES} 
# 										${YARP_LIBRARIES} 
# 										${libqpOASES}
# 										pthread
# 										interface
# 										solvers
# 										)
# -----------------------------------------------------------------------


# #########################################################################################################
SET(CMAKE_C_FLAGS "-D USE_USB_CONNECTION")
add_definitions(-D_USE_MATH_DEFINES)
set(CMAKE_CXX_FLAGS "-std=c++17")	
# code optimization
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")

# ###########
# add_library (Robot  			${ROBOT_SOURCES})
# add_library (invKinDyn  		${C_SOURCES} ${WBID_SOURCES} ${WBIK_SOURCES})
# add_library (BimControllers  	${C_BIMAN_SRC} ${BIMAN_SOURCES})

add_library (invKinDyn  		${WBID_SOURCES} ${WBIK_SOURCES})
add_library (BimControllers  	${BIMAN_SOURCES})

# add_library (solvers  	${C_SOURCES} ${C_BIMAN_SRC})

# target_link_libraries(Robot 	${iDynTree_LIBRARIES} ${YARP_LIBRARIES})
target_link_libraries(invKinDyn ${iDynTree_LIBRARIES} ${YARP_LIBRARIES} solvers)
target_link_libraries(BimControllers ${iDynTree_LIBRARIES} ${YARP_LIBRARIES} solvers)

# ####
# # -----------------------------------------------------------------------
# # Executable for the project
# # -----------------------------------------------------------------------
add_executable(${PROJECT_NAME} 		${SOURCES} 
									${CODES_SOURCES} 
									${STEPSGEN_SOURCES}
									${ANTICIP_SOURCES}
									)

target_link_libraries(${PROJECT_NAME} 	${iDynTree_LIBRARIES} 
										${YARP_LIBRARIES}
										${libqpOASES} 
										interface
										invKinDyn
										BimControllers
										solvers
# 										qpsolvers
										)
# -----------------------------------------------------------------------


										


