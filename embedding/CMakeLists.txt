add_executable(mpga_emergent_behavior main.cpp)
target_link_libraries(mpga_emergent_behavior
        buzz
        argos3plugin_simulator_buzz
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media
        mpga
        analysis
        mpga_emergent_behavior_loop_functions
        ${BUZZ_LIBRARY})
