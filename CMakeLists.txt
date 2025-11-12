cmake_minimum_required(VERSION 3.15)
project(tonton VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
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
