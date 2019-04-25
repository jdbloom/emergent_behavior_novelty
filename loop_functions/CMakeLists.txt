include_directories(${CMAKE_SOURCE_DIR})
message(${CMAKE_SOURCE_DIR})
# Compile MPGA library (generic library for multi-process genetic algorithm)
add_library(mpga SHARED
        mpga.h mpga.cpp
        mpga_loop_functions.h mpga_loop_functions.cpp)
target_link_libraries(mpga
        argos3core_simulator
        buzz
        argos3plugin_simulator_buzz)

if (NOT APPLE)
    target_link_libraries(mpga rt)
endif (NOT APPLE)

# Compile phototaxis example
add_library(mpga_emergent_behavior_loop_functions SHARED
        mpga_emergent_behavior_loop_functions.h mpga_emergent_behavior_loop_functions.cpp)

target_link_libraries(mpga_emergent_behavior_loop_functions
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_kheperaiv
        argos3plugin_simulator_media
        mpga
        analysis)
