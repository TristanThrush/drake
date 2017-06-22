if (lcm_FOUND)
    add_library_with_exports(LIB_NAME drakeBhpnDrakeInterface SOURCE_FILES
            lcm_utils/robot_state_lcm.cc simulation_utils/world_sim_tree_builder.cc simulation_utils/sim_diagram_builder.cc)
    target_link_libraries(drakeBhpnDrakeInterface
            drakeLcmSystem
            drakeLCMUtil
            drakeMultibodyParsers
            drakeRigidBodyPlant
            drakeSystemAnalysis
            drakeSystemFramework
            drakeSystemPrimitives
            drakeSystemControllers
            oracularStateEstimator
            gflags
            spruce)
    drake_install_libraries(drakeBhpnDrakeInterface)
    drake_install_headers(lcm_utils/robot_state_lcm.h
            simulation_utils/world_sim_tree_builder.h simulation_utils/sim_diagram_builder.h)
    drake_install_pkg_config_file(drake-bhpn-drake-interface
            TARGET drakeBhpnDrakeInterface
            LIBS
            -ldrakeBhpnDrakeInterface
            -lgflags
            REQUIRES
            drake-bhpn-drake-interface
            drake-lcm-system2
            drake-multibody-parsers
            drake-rigid-body-plant
            drake-sim-diagram-builder
            drake-system-analysis
            drake-system-framework
            drake-system-primitives
            oracular-state-estimator
            spruce)

    add_executable(simulation simulation.cc)
    target_link_libraries(simulation
            drakeBhpnDrakeInterface
            drakeLcmSystem
            drakeMultibodyParsers
            drakeRigidBodyPlant
            drakeSystemAnalysis
            drakeSystemControllers
            drakeTrajectories
            drakeIKoptions
            drakeRigidBodyConstraint
            drakeIK
            gflags)

endif ()
