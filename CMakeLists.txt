cmake_minimum_required(VERSION 3.10)


SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories(../../..)
include_directories(./include)

set(ApplicationName Monitor)

add_executable(${ApplicationName} 
	./Example/Mono_CameraDevice.cpp 
	./src/CameraDevice/CameraDevice.cpp
	./src/Monitor/System.cpp
	./src/Monitor/Config.cpp
	./src/Monitor/Tracking.cpp
	./src/Monitor/Camera.cpp
	)
target_link_libraries( ${ApplicationName} ${OpenCV_LIBS} -lMVSDK)
set_property(TARGET ${ApplicationName} PROPERTY CXX_STANDARD 11)
install(TARGETS ${ApplicationName} DESTINATION bin)

#if(ADD_PYTHON_EXAMPLES)
#	add_custom_command(
#        TARGET ${ApplicationName}
#		POST_BUILD
#       COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/*.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
#	)
#endif()
