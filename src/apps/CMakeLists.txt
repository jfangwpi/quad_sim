# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/remoteApi)


add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)

add_executable(communicate_test communicate_test.cpp)
target_link_libraries(communicate_test vrep_sim controller)