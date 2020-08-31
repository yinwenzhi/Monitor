cmake_minimum_required(VERSION 3.10)
project(Monitor_PROJECT)

SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

list( APPEND CMAKE_MODULE_PATH /home/emast/WorkSpace/install/g2o/cmake_modules)
set(G2O_ROOT /home/emast/WorkSpace/lib/g2o/) 

find_package(G2O REQUIRED)
IF(G2O_FOUND)
    include_directories(${G2O_INCLUDE_DIR})
	MESSAGE(STATUS, "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
	MESSAGE(STATUS, "G2O_INCLUDE_DIR: ${G2O_INCLUDE_DIR}")
	MESSAGE(STATUS, "G2O found: ${G2O_FOUND}")
ENDIF(G2O_FOUND)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(CSparse REQUIRED )

include_directories( 
		${PROJECT_SOURCE_DIR}/include
		${PROJECT_SOURCE_DIR}/include/Monitor
		${PROJECT_SOURCE_DIR}/include/CameraDevice
		${OpenCV_INCLUDE_DIRS}
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
		)



link_directories("/home/emast/WorkSpace/lib/g2o/lib")
set( THIRD_PARTY_LIBS
	${OpenCV_LIBS}
	${Sophus_LIBRARIES}
	${Pangolin_LIBRARIES}
	${CSPARSE_LIBRARY}
	${G2O_LIBRARIES}
	${G2O_LIBRARIE}
	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

add_library( ${PROJECT_NAME}  SHARED
	./src/CameraDevice/CameraDevice.cpp
	./src/Monitor/System.cpp
	./src/Monitor/Config.cpp
	./src/Monitor/Tracking.cpp
	./src/Monitor/Camera.cpp
	./src/Monitor/Feature.cpp
	./src/Monitor/Frame.cpp
	./src/Monitor/ORBmatcher.cpp
	)
target_link_libraries( ${PROJECT_NAME} 
	${THIRD_PARTY_LIBS} 
	-lMVSDK
	)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)	

set(ApplicationName Mono_CameraDevice)
# add_executable(${ApplicationName} ./Example/Mono_CameraDevice.cpp
# 	./src/CameraDevice/CameraDevice.cpp
# 	./src/Monitor/System.cpp
# 	./src/Monitor/Config.cpp
# 	./src/Monitor/Tracking.cpp
# 	./src/Monitor/Camera.cpp
# 	./src/Monitor/Feature.cpp
# 	./src/Monitor/Frame.cpp
# 	)
add_executable(${ApplicationName} ./Example/Mono_CameraDevice.cpp
	
	)
set_property(TARGET ${ApplicationName} PROPERTY CXX_STANDARD 11)
target_link_libraries(${ApplicationName} 
	${PROJECT_NAME} 
	
	)
#install(TARGETS ${ApplicationName} DESTINATION bin)


