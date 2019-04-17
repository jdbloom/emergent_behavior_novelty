add_executable(mpga_emergent_behavior main.cpp)
target_link_libraries(mpga_emergent_behavior
        mpga_emergent_behavior_loop_functions
        argos3core_simulator)
