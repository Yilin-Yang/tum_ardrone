set(GUI_SOURCE_FILES
    main_GUI.cpp
    tum_ardrone_gui.cpp
    RosThread.cpp
    PingThread.cpp
)

set(GUI_HEADER_FILES
    tum_ardrone_gui.h
    RosThread.h
    PingThread.h
)

# *.ui
set(GUI_UI_FILES
    tum_ardrone_gui.ui
)

# *.qrc
set(GUI_RESOURCE_FILES
)

# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# build!
add_executable(drone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(drone_gui ${QT_LIBRARIES} cvd ${catkin_LIBRARIES})
add_dependencies(drone_gui thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
