cmake_minimum_required( VERSION 2.8 )
project( rotation )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )


find_package( OpenCV  REQUIRED )


include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
   # "/usr/include/eigen3/"
)

add_executable( RtoEular testEular.cpp  )
target_link_libraries( RtoEular ${OpenCV_LIBS} )

