
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include_directories( ${IOKit_INCLUDE_DIRS} )

set( GAMEPAD_HDRS   Gamepad.h
                    EventDispatcher.h
                 )

set( GAMEPAD_SRC    EventDispatcher.cpp
                 )

# Test platform type
IF(_WIN_)
    set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_windows.c )
ENDIF()

IF(_LINUX_)
    set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_linux.c )
ENDIF()

IF(_OSX_)
    set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_macosx.cpp )
    set( GAMEPAD_LIBRARIES ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
ENDIF()

ADD_LIBRARY(Gamepad ${GAMEPAD_HDRS} ${GAMEPAD_SRC} )
TARGET_LINK_LIBRARIES(Gamepad ${GAMEPAD_LIBRARIES} ${IOKit_LIBRARIES} )



