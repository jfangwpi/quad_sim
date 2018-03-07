# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../vrep_sim/vrep_lib/remoteApi)


add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)

# Add libraries
set(CONTROLLER_LIB_SRC
    quad_controller.cpp
    quad_client.cpp
    quad_states.cpp
)

add_library(controller STATIC ${CONTROLLER_LIB_SRC})
#target_link_libraries(controller)


