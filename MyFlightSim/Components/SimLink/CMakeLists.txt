####
# F Prime CMakeLists.txt:
#
# SOURCES: list of source files (to be compiled)
# AUTOCODER_INPUTS: list of files to be passed to the autocoders
# DEPENDS: list of libraries that this module depends on
#
# More information in the F´ CMake API documentation:
# https://fprime.jpl.nasa.gov/latest/docs/reference/api/cmake/API/
#
####

register_fprime_library(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/SimLink.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/SimLink.cpp"
    DEPENDS
        # SimLink.cpp uses SimHeader and SIM_MAGIC. SimLink.fpp does not mention
        # them, so the build cannot discover this dependency on its own.
        MyFlightSim_Types
)

### Unit Tests ###
register_fprime_ut(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/SimLink.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/SimLinkTestMain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/SimLinkTester.cpp"
    DEPENDS
        MyFlightSim_Types
    UT_AUTO_HELPERS
)
