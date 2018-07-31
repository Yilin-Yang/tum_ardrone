set(TUM_ARDRONE_SRC_FOLDER
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# set required libs and headers
include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}

)

include_directories(
    ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/TooN/include
    ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include
    ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/include
)

link_directories(
    ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib
    ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/lib
)

add_subdirectory(autopilot)
add_subdirectory(stateestimation)
add_subdirectory(UINode)
