# ------------------------- autopilot & KI -----------------------------------------
set(AUTOPILOT_SOURCE_FILES
    main_autopilot.cpp
    ControlNode.cpp
    DroneController.cpp
    KI/KILand.cpp
    KI/KIAutoInit.cpp
    KI/KIFlyTo.cpp
)
set(AUTOPILOT_HEADER_FILES
    ControlNode.h
    DroneController.h
    KI/KILand.h
    KI/KIAutoInit.h
    KI/KIFlyTo.h
    KI/KIProcedure.h
)

# build!
add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
target_link_libraries(drone_autopilot ${catkin_LIBRARIES})
add_dependencies(drone_autopilot thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
