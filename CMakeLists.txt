cmake_minimum_required( VERSION 2.8 )

project( SituatedSIPP )

set(Boost_USE_STATIC_LIBS OFF) 
#find_package(Boost COMPONENTS multi_index)
include_directories( "/home/aifs1/gu/.conan/data/boost/1.74.0/_/_/package/8773c88b4cea0ec86a6e2f9053eb6a7a40608da3/include/boost" )
include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     main.cpp  
     tinyxml2.cpp
     xmlLogger.cpp 
     mission.cpp 
     map.cpp 
     task.cpp
     config.cpp 
     dynamicobstacles.cpp
     aa_sipp.cpp
     constraints.cpp )

set( HEADER_FILES 
     tinyxml2.h
     heuristic.h
     searchresult.h 
     gl_const.h 
     xmlLogger.h 
     mission.h 
     map.h 
     task.h
     dynamicobstacles.h
     config.h
     lineofsight.h
     aa_sipp.h 
     structs.h
     constraints.h )

add_executable( ssipp ${SOURCE_FILES} ${HEADER_FILES} )
