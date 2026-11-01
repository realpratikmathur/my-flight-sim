####
# Project-wide FPP types and ports shared by components and by the simulator.
# Module name (derived from the path): MyFlightSim_Types
####
register_fprime_module(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/SimTypes.fpp"
        "${CMAKE_CURRENT_LIST_DIR}/VehiclePorts.fpp"
)
