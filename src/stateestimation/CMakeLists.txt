# --------------------------- stateestimation & PTAM --------------------------------
# set header and source files
set(STATEESTIMATION_SOURCE_FILES
    GLWindow2.cc
    GLWindowMenu.cc
    main_stateestimation.cpp
    DroneKalmanFilter.cpp
    Predictor.cpp
    PTAMWrapper.cpp
    MapView.cpp
    EstimationNode.cpp
    PTAM/ATANCamera.cc
    PTAM/Bundle.cc
    PTAM/HomographyInit.cc
    PTAM/KeyFrame.cc
    PTAM/Map.cc
    PTAM/MapMaker.cc
    PTAM/MapPoint.cc
    PTAM/MiniPatch.cc
    PTAM/PatchFinder.cc
    PTAM/Relocaliser.cc
    PTAM/ShiTomasi.cc
    PTAM/SmallBlurryImage.cc
    PTAM/Tracker.cc
)
set(STATEESTIMATION_HEADER_FILES
    ${TUM_ARDRONE_SRC_FOLDER}/HelperFunctions.h
    GLWindow2.h
    GLWindowMenu.h
    MouseKeyHandler.h
    DroneKalmanFilter.h
    Predictor.h
    PTAMWrapper.h
    MapView.h
    EstimationNode.h
    PTAM/ATANCamera.h
    PTAM/Bundle.h
    PTAM/customFixes.h
    PTAM/HomographyInit.h
    PTAM/KeyFrame.h
    PTAM/LevelHelpers.h
    PTAM/Map.h
    PTAM/MapMaker.h
    PTAM/MapPoint.h
    PTAM/MEstimator.h
    PTAM/MiniPatch.h
    PTAM/OpenGL.h
    PTAM/PatchFinder.h
    PTAM/Relocaliser.h
    PTAM/settingsCustom.h
    PTAM/ShiTomasi.h
    PTAM/SmallBlurryImage.h
    PTAM/SmallMatrixOpts.h
    PTAM/TrackerData.h
    PTAM/Tracker.h
    PTAM/VideoSource.h
)

set(PTAM_LIBRARIES GL glut cvd GVars3 blas lapack)
add_definitions(-DKF_REPROJ)

# build!
add_executable(drone_stateestimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})
set_target_properties(drone_stateestimation PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3")
target_link_libraries(drone_stateestimation ${PTAM_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(drone_stateestimation thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
