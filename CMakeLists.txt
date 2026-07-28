cmake_minimum_required(VERSION 3.15)
project(tonton VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add submodules
add_subdirectory(modules/dodeedum)

# Glob source files
file(GLOB_RECURSE TONTON_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Glob header files for IDE support
file(GLOB_RECURSE TONTON_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
)

# Create static library
add_library(tonton STATIC
    ${TONTON_SOURCES}
    ${TONTON_HEADERS}
)

# Make an unhandled enumerator a HARD ERROR, not a silent fall-through.
#
# src/Control/tonton_envelope.cpp (ExtractEnvelope) and src/Control/tonton_myopic.cpp
# (SpeedFrameOf) both deliberately omit `default:` from their switch over
# LocomotionMode, and both say in a comment that adding a mode is therefore a
# compile error. Without this flag that claim is simply false: the project builds
# with plain -std=c++20, no -Wall and no -Werror, so a seventh enumerator compiles
# with ZERO diagnostics and silently gets `return std::nullopt` and the GROUND
# speed frame -- the exact silent-wrong-answer outcome the missing `default:`
# exists to prevent. Even -Wall makes it only a warning.
#
# -Werror=switch fires ONLY on a switch over an enum that has no `default:` label
# and does not name every enumerator, so switches that already carry a `default:`
# are unaffected by it.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(tonton PRIVATE -Werror=switch)
endif()

# Set include directories
target_include_directories(tonton
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link with dodeedum submodule
target_link_libraries(tonton
    PUBLIC
        dodeedum
)

# Force tonton to have a direct dependency on dodeedum's library file
# This ensures CMake properly tracks when dodeedum.a changes
add_dependencies(tonton dodeedum)

# Find and link GLM (if not already found by dodeedum)
find_package(glm QUIET)
if(glm_FOUND)
    target_link_libraries(tonton PUBLIC glm::glm)
else()
    message(STATUS "GLM not found via find_package, assuming it's in include path")
endif()

# Set library properties
set_target_properties(tonton PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

# Optional: Install rules (uncomment if needed)
# install(TARGETS tonton
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     PUBLIC_HEADER DESTINATION include/tonton
# )
