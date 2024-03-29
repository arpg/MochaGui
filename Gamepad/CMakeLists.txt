project(Gamepad)
cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

set( GAMEPAD_HDRS   Gamepad.h
                    EventDispatcher.h
                 )

set( GAMEPAD_SRC    EventDispatcher.cpp
                 )

# test platform type
if(WIN32)
    set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_windows.c )
endif()

if(UNIX)
    if(_OSX_)
				find_package( IOKit REQUIRED )
				include_directories( ${IOKit_INCLUDE_DIRS} )
				find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
				message("Finding library CoreFoundation: ${COREFOUNDATION_FRAMEWORK}")
				set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_macosx.cpp )
    else()
        set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_linux.cpp )
    endif()
endif()

add_library(Gamepad ${GAMEPAD_HDRS} ${GAMEPAD_SRC} )
target_link_libraries(Gamepad ${IOKit_LIBRARIES} ${COREFOUNDATION_FRAMEWORK})
