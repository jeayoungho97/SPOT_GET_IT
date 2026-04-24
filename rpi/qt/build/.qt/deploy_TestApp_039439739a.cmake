include(/home/pi/robot_project/qt_app/build/.qt/QtDeploySupport.cmake)
include("${CMAKE_CURRENT_LIST_DIR}/TestApp-plugins.cmake" OPTIONAL)
set(__QT_DEPLOY_I18N_CATALOGS "qtbase")

qt6_deploy_runtime_dependencies(
    EXECUTABLE /home/pi/robot_project/qt_app/build/TestApp
    GENERATE_QT_CONF
)
