##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(zl_Crane_AutomaticLift_trunk)

##############################################################################
# Catkin
##############################################################################
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")
add_compile_options(-Wall -std=c++14 -Wunused-variable -O3)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt5(QtCore QtGui) # Add the appropriate components to the component list here

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED )

find_package(zl_manipulation CONFIG REQUIRED 
    # PATHS "${CMAKE_CURRENT_SOURCE_DIR}/3rd_partys/pathplan/zl_manipulation/build"
    # NO_DEFAULT_PATH  # 强制只在这个路径找，防止找到系统旧版本
)
find_package(zl_motion_planning CONFIG REQUIRED 
    # PATHS "${CMAKE_CURRENT_SOURCE_DIR}/3rd_partys/pathplan/zl_motion_planning/build"
    # NO_DEFAULT_PATH  # 强制只在这个路径找，防止找到系统旧版本
)
find_package(orocos_kdl 1.4 REQUIRED
    PATHS /home/xpp/3rdpartlibs/orocos-kdl-release-upstream-1.4.0/orocos_kdl/build
    NO_DEFAULT_PATH
)
find_package(OpenGL REQUIRED)
find_package(FreeGLUT REQUIRED
    PATHS /home/xpp/3rdpartlibs/freeglut-master/build/FreeGLUT
    NO_DEFAULT_PATH
)
find_package(VTK 6.3.0 EXACT REQUIRED)
find_package(PCL 1.8.1 EXACT REQUIRED)
find_package(Protobuf REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(Torch REQUIRED
#         PATHS /usr/local/libtorch/share/cmake
#         NO_DEFAULT_PATH)
# find_package(siamrpn CONFIG REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON) #自动添加 CMAKE_CURRENT_BINARY_DIR 和 CMAKE_CURRENT_SOURCE_DIR 到当前处理的 CMakeLists.txt
##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/zl_Crane_AutomaticLift_trunk/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

set(OpenCV_DIR /usr/local/opencv320/share/OpenCV)
set(HCNetSDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_partys/HCNetSDK)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED OpenCV)
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    roslib
    rospy
    std_msgs
    objects_msg
    cv_bridge
    image_transport
    HookLoadAntiCollisionDetection
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package(
    CATKIN_DEPENDS
    objects_msg
    )

##############################################################################
# Binaries
##############################################################################
#把该路径添加到第三方库搜索路径中

MESSAGE(STATUS "PROJECT_NAME: ${PROJECT_NAME}")
include_directories(
    include/zl_Crane_AutomaticLift_trunk
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${HCNetSDK_DIR}/include
    ${orocos_kdl_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${FreeGLUT_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${Torch_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
MESSAGE(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP}")

#检测器需要的库
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${FreeGLUT_LIBRARIES}
    # ${TORCH_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    zl_motion_planning::zl_motion_planning
    zl_manipulation::zl_manipulation
    # siamrpn::siamrpn
    paho-mqtt3a
    paho-mqtt3as
    paho-mqttpp3
    ${PROTOBUF_LIBRARIES}
    )

# 2. 扫描该目录下所有的动态库文件，存入变量 ALL_LIBS
file(GLOB HCNetSDK_LIBS "${HCNetSDK_DIR}/lib/*.so*")
file(GLOB HCNetSDKCom_LIBS "${HCNetSDK_DIR}/lib/HCNetSDKCom/*.so*")


# 4. 链接所有找到的库
target_link_libraries(${PROJECT_NAME} 
    ${HCNetSDKCom_LIBS}
    ${HCNetSDK_LIBS}
)
target_link_libraries(${PROJECT_NAME} libpcanbasic.so)

# === 运行时路径配置 ===
set_target_properties(${PROJECT_NAME} PROPERTIES 
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${HCNetSDK_DIR}/lib;${HCNetSDK_DIR}/lib/HCNetSDKCom"
)
# ======================


#install(FILES src/HPconfig.ini DESTINATION /etc/${PROJECT_NAME}/config COMPONENT config)


#add_executable(zl_Crane_AutomaticLift_trunk ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(zl_Crane_AutomaticLift_trunk ${QT_LIBRARIES} ${catkin_LIBRARIES})
#install(TARGETS zl_Crane_AutomaticLift_trunk RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

