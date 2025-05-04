cmake_minimum_required(VERSION 3.15)
project(TrafficSimulation VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find required packages
find_package(pybind11 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    ${pybind11_INCLUDE_DIRS}
)

# Collect source files
file(GLOB_RECURSE CPP_SOURCES 
    "cpp/src/*.cpp"
)

# Add library target
add_library(traffic_simulation STATIC ${CPP_SOURCES})

# Compiler warnings and options
target_compile_options(traffic_simulation PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

# Link dependencies
target_link_libraries(traffic_simulation PUBLIC
    nlohmann_json::nlohmann_json
)

# Python bindings
file(GLOB_RECURSE PYBIND_SOURCES 
    "cpp/binding/*.cpp"
)

pybind11_add_module(traffic_system ${PYBIND_SOURCES})
target_link_libraries(traffic_system PRIVATE traffic_simulation)

# Add test executable
file(GLOB_RECURSE TEST_SOURCES 
    "tests/cpp_tests/*.cpp"
)

if(TEST_SOURCES)
    add_executable(traffic_tests ${TEST_SOURCES})
    target_link_libraries(traffic_tests PRIVATE traffic_simulation)
endif()

# Installation
install(TARGETS traffic_simulation
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(TARGETS traffic_system
    LIBRARY DESTINATION lib
)

install(DIRECTORY cpp/include/
    DESTINATION include/traffic_simulation
    FILES_MATCHING PATTERN "*.h"
)

# Python setup
configure_file(build_scripts/setup.py.in setup.py @ONLY)

# Copy Python modules
file(COPY python/traffic_simulation DESTINATION ${CMAKE_BINARY_DIR})

# Add custom target for building Python package
add_custom_target(python_package
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/python/traffic_simulation
        ${CMAKE_BINARY_DIR}/traffic_simulation
    COMMAND ${Python_EXECUTABLE} -m pip install -e .
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS traffic_system
    COMMENT "Building Python package"
)

# Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_INPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include)
    set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/docs)
    set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)
    set(DOXYGEN_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    
    configure_file(docs/Doxyfile.in ${DOXYGEN_CONFIG_FILE} @ONLY)
    
    add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIG_FILE}
        MAIN_DEPENDENCY ${DOXYGEN_CONFIG_FILE}
        DEPENDS ${CPP_SOURCES}
        COMMENT "Generating documentation with Doxygen"
    )
    
    add_custom_target(docs
        DEPENDS ${DOXYGEN_INDEX_FILE}
    )
endif()

# Print configuration summary
message(STATUS "Configuration Summary:")
message(STATUS "  CMAKE_GENERATOR: ${CMAKE_GENERATOR}")
message(STATUS "  CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "  CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  pybind11 version: ${pybind11_VERSION}")
message(STATUS "  nlohmann_json version: ${nlohmann_json_VERSION}")
message(STATUS "  Build will be output to: ${CMAKE_BINARY_DIR}")