project( mochapc )
cmake_minimum_required(VERSION 2.8)

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

# Test platform type
IF(WIN32 OR WIN64)
    MESSAGE("Windows system found..")
  SET(_WIN_ 1)
ENDIF()

IF(UNIX)
  MESSAGE("Unix system found..")
  SET(_UNIX_ 1)
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE("OS X system found..")
   SET(_OSX_ 1)
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    MESSAGE("Linux system found..")
   SET(_LINUX_ 1)
ENDIF()

# TODO fix all these warnings instead of suppressing them!!

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations -Wno-unused-private-field -Wno-unused-value -Wno-overloaded-virtual -fPIC" )
add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

# Add to module path, so we can find our cmake modules

#the boost libraries that we need
set(BOOST_LIBS thread date_time system timer filesystem regex )


#this dependency is required for newer versions of the boost library. unfortunately
#boost doesn't seem to link to it..

#find_package( Pangolin REQUIRED)
find_package( Eigen3 REQUIRED )
find_package( GLog REQUIRED)
# find_package( carplanner REQUIRED )
#find_package( CarPlanner REQUIRED )
#find_package( SceneGraph REQUIRED )
find_package( ZeroMQ REQUIRED )
# find_package( CVars REQUIRED )
find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
#find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
# find_package( CeresSolver REQUIRED )
#find_package( SensorFusion REQUIRED )
find_package( LibAO REQUIRED )
if(_OSX_)
  find_package( IOKit REQUIRED )
endif()
find_package( Sophus REQUIRED )
find_package( assimp REQUIRED )
#find_package( Node REQUIRED )
#find_package( pcl REQUIRED )
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    tf
    carplanner_msgs
    pcl_ros
    pcl_conversions
    nav_msgs
    nodelet
    mesh_msgs
    actionlib
    # mt_actionlib
    )

add_definitions(${EIGEN_DEFINITIONS})

#generate_messages(
#  DEPENDENCIES
#    carplanner_msgs
#    std_msgs
#)

catkin_package(
  INCLUDE_DIRS .
  CATKIN_DEPENDS roscpp std_msgs tf carplanner_msgs
  )

# link_directories( ${Pangolin_LIBRARY_DIRS} )
link_directories( ${Boost_LIBRARY_DIRS} )

set( NV_CMAKE_DIR ${CMAKE_MODULE_PATH} )
#add_subdirectory( NvWidgets )
# add_subdirectory( Gamepad )

include_directories( ${CMAKE_BINARY_DIR} )
#PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS CarMessages.proto )


set( _INCLUDE_DIRS
	    ${GLog_INCLUDE_DIRS}
            #${Node_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
            ${LIBAO_INCLUDE_DIR}
            #${Pangolin_INCLUDE_DIRS}
            ${BOOST_INCLUDE_DIR}
	    # ${carplanner_INCLUDE_DIRS}
	    #${CarPlanner_INCLUDE_DIRS}
	    # ${CVars_INCLUDE_DIRS}
            ${ZeroMQ_INCLUDE_DIRS}
            #${PROTOBUF_INCLUDE_DIRS}
            #${SceneGraph_INCLUDE_DIRS}
            ${BULLET_INCLUDE_DIR}
            ${CeresSolver_INCLUDE_DIRS}
            #${SensorFusion_INCLUDE_DIRS}
            ${assimp_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_BINARY_DIR}
            ${catkin_INCLUDE_DIRS}
       )

set( LIBRARIES
               ${GLog_LIBRARIES}
               #${Node_LIBRARIES}
	${BULLET_LIBRARIES}
               ${CeresSolver_LIBRARIES}
               #${SensorFusion_LIBRARIES}
               #${Pangolin_LIBRARIES}
               ${LIBAO_LIBRARY}
               #${SceneGraph_LIBRARIES}
	      #  ${carplanner_LIBRARIES}
	       #${CarPlanner_LIBRARIES}
	       ${ZeroMQ_LIBRARIES}
         #${PROTOBUF_LIBRARIES}
               ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_TIMER_LIBRARY}  #link in the boost libraries we want
              #  ${CVars_LIBRARIES}
               #nvwidgets #this is built with MochaGui. No finding is required
              #  Gamepad # this is also built with the solution
               ${assimp_LIBRARIES}
               ${catkin_LIBRARIES}
        )

    message(STATUS ${assimp_LIBRARIES})

include_directories( ${_INCLUDE_DIRS} )

set( SHARED_HDRS 
  # PlannerGui.h
                #  GLBulletDebugDrawer.h
                #  UiCommon.h
                #  ProcessModelFusion.h
                #  GLCar.h
                 #EventLogger.h
                #  SensorFusionCeres.h
                 #Vicon.h
                 BoundarySolver.h
                 BezierBoundarySolver.h
                 CarPlannerCommon.h
                 CarParameters.h
                #  CVarHelpers.h
                 RpgUtils.h
                 MochaProblem.h
		            mesh_conversion_tools.hpp
                tf_conversion_tools.hpp
                RaycastVehicle.h
                WheelInfo.h
                 )

set( SHARED_SRC
              #  PlannerGui.cpp
                # GLBulletDebugDrawer.cpp
                # ProcessModelFusion.cpp
                # JoystickHandler.cpp
                #EventLogger.cpp
                #CarMessages.pb.cc
                # SensorFusionCeres.cpp
                #Vicon.cpp
                 CarParameters.cpp
                 BezierBoundarySolver.cpp
                 MochaProblem.cpp
                 RaycastVehicle.cpp
                 WheelInfo.cpp
                )


set( GUI_HDRS   config.h.in
                GLGuiPanel.h
                MochaGui.h
                 )

set( GUI_SRC    MochaGui.cpp
                 )


set( LEARNING_HDRS  config.h.in
                    JoystickHandler.h
                    GLLearningPanel.h
                    LearningGui.h )
set( LEARNING_SRC
                  LearningGui.cpp )

#add_executable( fake_mesh_publisher fake_mesh_publisher.cpp ${SHARED_HDRS} ${SHARED_SRC} ${catkin_INCLUDE_DIRS} )
#target_link_libraries( fake_mesh_publisher ${LIBRARIES} ${catkin_LIBRARIES} )

#add_executable( command2cmdvel Command2CmdVel.cpp ${SHARED_HDRS} ${SHARED_SRC} ${catkin_INCLUDE_DIRS} )
#target_link_libraries( command2cmdvel ${LIBRARIES} ${catkin_LIBRARIES} )

#add_executable(  logger applications/logger.cpp CarMessages.pb.cc ${SHARED_HDRS} ${SHARED_SRC} )
#target_link_libraries( logger ${LIBRARIES} )

#add_executable(  refplane applications/refplane.cpp ${SHARED_HDRS} ${SHARED_SRC} )
#target_link_libraries( refplane ${LIBRARIES} )

#add_executable(  learning applications/learning.cpp ${SHARED_HDRS} ${LEARNING_HDRS} ${SHARED_SRC} ${LEARNING_SRC} )
#target_link_libraries( learning ${LIBRARIES} )

#add_executable(  opt applications/opt.cpp ${SHARED_HDRS} ${SHARED_SRC} )
#target_link_libraries( opt ${LIBRARIES} )

#add_executable(  control-daemon applications/control-daemon.cpp ${SHARED_HDRS} ${SHARED_SRC}  )
#target_link_libraries( control-daemon ${LIBRARIES} )

#add_executable( fusion applications/fusion.cpp ${SHARED_HDRS} ${SHARED_SRC}  )
#target_link_libraries( fusion ${LIBRARIES} )

#add_executable(  gui mochagui_node.cpp ${SHARED_HDRS} ${GUI_HDRS} ${SHARED_SRC} ${GUI_SRC} )
#target_link_libraries( gui ${LIBRARIES} )

### planner libraries ###
add_library( planner
                ${SHARED_HDRS}
                MochaPlanner.h
                ${SHARED_SRC}
                MochaPlanner.cpp)
target_link_libraries(planner ${LIBRARIES})
add_dependencies( planner
  carplanner_msgs_gencfg
  carplanner_msgs_gencfg
)

add_library( planner_nodelet MochaPlannerNodelet.cpp)
target_link_libraries( planner_nodelet planner ${LIBRARIES} )

add_executable(  planner_standalone_nodelet MochaPlannerStandaloneNodelet.cpp )
target_link_libraries( planner_standalone_nodelet planner_nodelet )

### vehicle libraries ###
add_library( vehicle
                ${SHARED_HDRS}
                MochaVehicle.h
                ${SHARED_SRC}
                MochaVehicle.cpp)
target_link_libraries(vehicle ${LIBRARIES})
#add_dependencies(vehicle vehicle_gencfg)

add_library( vehicle_nodelet MochaVehicleNodelet.cpp)
target_link_libraries( vehicle_nodelet vehicle ${LIBRARIES} )

add_executable(  vehicle_standalone_nodelet MochaVehicleStandaloneNodelet.cpp )
target_link_libraries( vehicle_standalone_nodelet vehicle_nodelet )

### controller ###
add_library( controller
                ${SHARED_HDRS}
		            MochaController.h
                ${SHARED_SRC}
				        MochaController.cpp)
target_link_libraries(controller ${LIBRARIES})

add_library( controller_nodelet MochaControllerNodelet.cpp)
target_link_libraries( controller_nodelet controller ${LIBRARIES} )

add_executable(  controller_standalone_nodelet MochaControllerStandaloneNodelet.cpp )
target_link_libraries( controller_standalone_nodelet controller_nodelet )

### simulator ###
#add_executable( simulator
#  MochaSimulator.cpp
#)
#target_link_libraries( simulator
#  vehicle
#)

# Generate config file for use in headers
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)


install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
