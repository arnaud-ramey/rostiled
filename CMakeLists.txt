### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(rostiled)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(DEPS  roscpp  roslib  std_msgs)
find_package(catkin REQUIRED COMPONENTS ${DEPS})
# includes cmake/FindSDL2.cmake
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
# nothing to do in this package
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
### 5 - Invoke message/service/action generation (generate_messages())
# nothing to do in this package
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS   include
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(include  ${catkin_INCLUDE_DIRS})
add_subdirectory(src/tmx)
include_directories(src/tmx/src)

SET(PKG_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/rostiled)
add_executable(sdl  src/sdl.cpp
                    ${PKG_INC_DIR}/tiled2sdl.h
                    ${PKG_INC_DIR}/utils.h)
TARGET_LINK_LIBRARIES(sdl ${catkin_LIBRARIES}
                          ${SDL2_LIBRARY} SDL2_image tmx  xml2  z)

add_executable(rostiled2sdl  src/rostiled2sdl.cpp)
TARGET_LINK_LIBRARIES(rostiled2sdl ${catkin_LIBRARIES}
                          ${SDL2_LIBRARY} SDL2_image tmx  xml2  z)

### 8 - Tests to build (catkin_add_gtest())
# nothing to do in this package
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
