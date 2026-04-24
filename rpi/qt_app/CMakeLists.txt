cmake_minimum_required(VERSION 3.19)
project(TestApp LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Svg OpenGL OpenGLWidgets)

set(CMAKE_AUTORCC ON)
qt_standard_project_setup()

qt_add_executable(TestApp
    WIN32 MACOSX_BUNDLE
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/shm_reader.cpp
    src/shm_reader.h
    src/pointcloudwidget.cpp
    src/pointcloudwidget.h
    src/lidarmap2dwidget.cpp
    src/lidarmap2dwidget.h
    src/mainwindow.ui
    resources/resources.qrc
)

target_link_libraries(TestApp
    PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Svg
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        pthread
        rt
        GL
)

target_include_directories(TestApp PRIVATE ../bridge_app)

install(TARGETS TestApp
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET TestApp
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
