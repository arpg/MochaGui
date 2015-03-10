project( MochaGui )
cmake_minimum_required(VERSION 2.8)

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


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")
add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

#the boost libraries that we need
set(BOOST_LIBS regex thread date_time system timer  )


#this dependency is required for newer versions of the boost library. unfortunately
#boost doesn't seem to link to it..
find_package(ICU COMPONENTS data uc i18n io le lx REQUIRED)
find_package(Pangolin REQUIRED)
find_package( CarPlanner REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( ZeroMQ REQUIRED )
find_package( MVL REQUIRED )
find_package( CVARS REQUIRED )
find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
find_package( SensorFusion REQUIRED )
find_package( LibAO REQUIRED )
find_package( IOKit REQUIRED )
find_package(Sophus REQUIRED)

link_directories( ${Pangolin_LIBRARY_DIRS} )
link_directories( ${Boost_LIBRARY_DIRS} )

set( NV_CMAKE_DIR ${CMAKE_MODULE_PATH} )
add_subdirectory(NvWidgets)
add_subdirectory(Gamepad)


include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Messages.proto )


set( _INCLUDE_DIRS
            ${Sophus_INCLUDE_DIR}
            ${LIBAO_INCLUDE_DIR}
            ${Pangolin_INCLUDE_DIRS}
            ${BOOST_INCLUDE_DIR} 
            ${CAR_PLANNER_INCLUDE_DIRS} 
            ${MVL_INCLUDE_DIRS} 
            ${CVARS_INCLUDE_DIR} 
            ${ZeroMQ_INCLUDE_DIRS} 
            ${PROTOBUF_INCLUDE_DIRS} 
            ${SceneGraph_INCLUDE_DIRS} 
            ${BULLET_INCLUDE_DIR} 
            ${SENSORFUSION_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_BINARY_DIR}
       )

set( LIBRARIES ${ICU_LIBRARIES}
               ${SENSORFUSION_LIBRARIES}
               ${Pangolin_LIBRARIES}
               ${LIBAO_LIBRARY}
               ${SceneGraph_LIBRARIES}
               ${MVL_LIBRARIES}
               ${CAR_PLANNER_LIBRARIES}
               ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES}
               ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_TIMER_LIBRARY}  #link in the boost libraries we want
               ${CVARS_LIBRARIES}
               nvwidgets #this is built with MochaGui. No finding is required
               Gamepad # this is also built with the solution
        )

include_directories( ${_INCLUDE_DIRS} )

set( SHARED_HDRS PlannerGui.h
                 GLBulletDebugDrawer.h
                 UiCommon.h
                 ProcessModelFusion.h
                GLCar.h
                EventLogger.h
                )

set( SHARED_SRC PlannerGui.cpp
                GLBulletDebugDrawer.cpp
                ProcessModelFusion.cpp
                JoystickHandler.cpp
                EventLogger.cpp
                Messages.pb.cc
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


add_executable(  logger applications/logger.cpp Messages.pb.cc ${SHARED_HDRS} ${SHARED_SRC} )
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



# Generate config file for use in headers
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

