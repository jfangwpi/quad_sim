cmake_minimum_required(VERSION 2.8.11)
project(quad_sim)

## Set compiler to use c++ 11 features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Choose to enable g3log or not: on/off
#set(ENABLE_LOGGING on)

#if(ENABLE_LOGGING)
#	message( "-- Enable logging with g3log -- Yes" )
#	message( "-- Deafult log directory: quad/control/log " )
#	include_directories(${CMAKE_SOURCE_DIR}/src/third_party/g3log/src)
#else(ENABLE_LOGGING)
#	message( "-- Enable logging with g3log -- No" )
#endif(ENABLE_LOGGING)


include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

## Add sub source directories
add_subdirectory(vrep_sim)
add_subdirectory(controller)
#add_subdirectory(third_party)
#add_subdirectory(logging)

