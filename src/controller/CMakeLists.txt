# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/remoteApi)


# Add libraries


#add_library(controller STATIC ${CONTROLLER_LIB_SRC})
#if (ENABLE_LOGGING)
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
#target_link_libraries(controller vrep_sim ${LCM_LIBRARIES})
#else (ENABLE_LOGGING)
#	add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
#	target_link_libraries(controller vrep_sim ${LCM_LIBRARIES})
#endif (ENABLE_LOGGING)

#add_executable(quadrotor_control quadrotor_control.cpp)
#target_link_libraries(quadrotor_control vrep_sim logging)
