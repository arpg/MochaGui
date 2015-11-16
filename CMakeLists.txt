project( MochaGui )
cmake_minimum_required(VERSION 2.8)

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Debug)
endif()

# TODO fix all these warnings instead of suppressing them!!

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations -Wno-unused-private-field -Wno-unused-value -Wno-overloaded-virtual" )
add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

include( SetPlatformVars )

#this dependency is required for newer versions of the boost library.

find_package( Pangolin REQUIRED)
find_package( GLOG REQUIRED)
find_package( CarPlanner REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
find_package( SensorFusion REQUIRED )
find_package( LibAO REQUIRED )
find_package( IOKit REQUIRED )
find_package( Sophus REQUIRED )
find_package( Node REQUIRED )

link_directories( ${Pangolin_LIBRARY_DIRS} )

#set( NV_CMAKE_DIR ${CMAKE_MODULE_PATH} )
#add_subdirectory(NvWidgets)
add_subdirectory(Gamepad)


include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    )

set( PROTO_DIR src/Messages )
set( _PROTO_SRCS
    # unique messages for MochaGui
    ${PROTO_DIR}/Messages.proto
    ${PROTO_DIR}/State.proto
    # messages from HAL
    ${PROTO_DIR}/Matrix.proto #from Matrix.proto
    ${PROTO_DIR}/Pose.proto
    ${PROTO_DIR}/Imu.proto
    )

PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS ${_PROTO_SRCS} )

set( _INCLUDE_DIRS
    ${GLOG_INCLUDE_DIRS}
    ${Node_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${LIBAO_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${CarPlanner_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${SceneGraph_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIR}
    ${SENSORFUSION_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}
    )

set( LIBRARIES
     ${Node_LIBRARIES}
     ${SENSORFUSION_LIBRARIES}
     ${Pangolin_LIBRARIES}
     ${LIBAO_LIBRARY}
     ${SceneGraph_LIBRARIES}
     ${CarPlanner_LIBRARIES}
     ${ZeroMQ_LIBRARIES}
     ${PROTOBUF_LIBRARIES}
     ${ASSIMP_LIBRARIES}
     ${BULLET_LIBRARIES}
     # nvwidgets #this is built with MochaGui. No finding is required
     Gamepad # this is also built with the solution
     )

include_directories( ${_INCLUDE_DIRS} )

set( SHARED_HDRS
    include/MochaGui/PlannerGui.h
    include/MochaGui/GLBulletDebugDrawer.h
    include/MochaGui/UiCommon.h
    include/MochaGui/ProcessModelFusion.h
    include/MochaGui/GLCar.h
    include/MochaGui/EventLogger.h
    include/MochaGui/SE3.h
    include/MochaGui/GetPot
    include/MochaGui/Localizer.h
    include/MochaGui/Matrix.h
    config.h.in
   )

list(APPEND SHARED_HDRS ${PROTO_HDRS})

set( SHARED_SRC
    src/PlannerGui.cpp
    src/GLBulletDebugDrawer.cpp
    src/ProcessModelFusion.cpp
    src/JoystickHandler.cpp
    src/EventLogger.cpp
    src/Localizer.cpp
    src/VarHelpers.h
    src/RpgUtils.h
   )

list(APPEND SHARED_SRC ${PROTO_SRCS})


set( GUI_HDRS
    include/MochaGui/gui/MochaGui.h
   )

set( GUI_SRC
    src/MochaGui.cpp
   )


set( LEARNING_HDRS
    include/MochaGui/learning/JoystickHandler.h
    include/MochaGui/learning/LearningGui.h
   )

set( LEARNING_SRC
     src/LearningGui.cpp
    )


add_executable(  logger applications/logger.cpp ${SHARED_HDRS} ${SHARED_SRC} )
target_link_libraries( logger ${LIBRARIES} )

add_executable(  refplane applications/refplane.cpp ${SHARED_HDRS} ${SHARED_SRC} )
target_link_libraries( refplane ${LIBRARIES} )

add_executable(  learning applications/learning.cpp ${SHARED_HDRS} ${LEARNING_HDRS} ${SHARED_SRC} ${LEARNING_SRC} )
target_link_libraries( learning ${LIBRARIES} )

add_executable(  opt applications/opt.cpp ${SHARED_HDRS} ${SHARED_SRC} )
target_link_libraries( opt ${LIBRARIES} )

add_executable(  gui applications/gui.cpp ${SHARED_HDRS} ${GUI_HDRS} ${SHARED_SRC} ${GUI_SRC} )
target_link_libraries( gui ${LIBRARIES} )

add_executable(  control-daemon applications/control-daemon.cpp ${SHARED_HDRS} ${SHARED_SRC}  )
target_link_libraries( control-daemon ${LIBRARIES} )

add_executable( fusion applications/fusion.cpp ${SHARED_HDRS} ${SHARED_SRC}  )
target_link_libraries( fusion ${LIBRARIES} )

# move resource files from source to build dir
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/herbie DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/jump.ply DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/learning_params.csv DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/gui_params.csv DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

# Generate config file for use in headers
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

