cmake_minimum_required(VERSION 2.8.12)
project(emergent_behavior)

if(APPLE)
  # Add address sanitizer support for CLang
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
endif(APPLE)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

# Find Lua
find_package(Lua52)
if(Lua52_FOUND)
    include_directories(${LUA_INCLUDE_DIR})
endif(Lua52_FOUND)


# Check for Buzz
find_package(Buzz REQUIRED)
include_directories(${BUZZ_C_INCLUDE_DIR})

# The sources 
set(SOURCE_FILES loop_functions/emergent_behavior.cpp loop_functions/emergent_behavior.h)

# Compile code
add_library(emergent_behavior SHARED ${SOURCE_FILES})
target_link_libraries(emergent_behavior
  buzz
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_qtopengl
  argos3plugin_simulator_buzz)
