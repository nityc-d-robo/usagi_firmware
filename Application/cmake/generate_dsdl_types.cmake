# Cyphal type generation using nunavut/pydsdl.
#
# Invokes nunavut twice (once per root namespace) and writes generated C++ headers into
# ${CMAKE_CURRENT_BINARY_DIR}/dsdl_types/.  A stamp file tracks build freshness so
# re-generation only happens when .dsdl sources change.
#
# Usage:
#   include(Application/cmake/generate_dsdl_types.cmake)
#

find_package(Python3 REQUIRED)
set(PYTHON_VENV_DIR "${CMAKE_CURRENT_BINARY_DIR}/.venv")

# 1. Create venv during CMake configure phase
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m venv "${PYTHON_VENV_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

# 2. Find Python executable in venv across operating systems
find_program(VENV_PYTHON
    NAMES python python3
    PATHS "${PYTHON_VENV_DIR}/bin" "${PYTHON_VENV_DIR}/Scripts"
    NO_DEFAULT_PATH
)

# 3. Install dependencies during configure phase
execute_process(
    COMMAND "${VENV_PYTHON}" -m pip install -q nunavut pydsdl jinja2 typeguard
)

set(DSDL_ROOT     "${CMAKE_SOURCE_DIR}/Drivers/public_regulated_data_types")
set(DSDL_TYPES_DIR   "${CMAKE_CURRENT_BINARY_DIR}/dsdl_types")
set(DSDL_TYPES_STAMP "${DSDL_TYPES_DIR}/.nunavut_stamp")

file(GLOB_RECURSE _DSDL_SOURCES
    "${DSDL_ROOT}/reg/*.dsdl"
    "${DSDL_ROOT}/uavcan/*.dsdl"
)

add_custom_command(
    OUTPUT "${DSDL_TYPES_STAMP}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${DSDL_TYPES_DIR}"
    # reg namespace (uavcan is a dependency lookup)
    COMMAND "${VENV_PYTHON}" -m nunavut
            --experimental-languages
            --target-language cpp
            --language-standard c++17
            --outdir "${DSDL_TYPES_DIR}"
            --lookup-dir "${DSDL_ROOT}/uavcan"
            "${DSDL_ROOT}/reg"
    # uavcan namespace (reg is a dependency lookup)
    COMMAND "${VENV_PYTHON}" -m nunavut
            --experimental-languages
            --target-language cpp
            --language-standard c++17
            --outdir "${DSDL_TYPES_DIR}"
            --lookup-dir "${DSDL_ROOT}/reg"
            "${DSDL_ROOT}/uavcan"
    COMMAND "${CMAKE_COMMAND}" -E touch "${DSDL_TYPES_STAMP}"
    DEPENDS ${_DSDL_SOURCES}
    COMMENT "Generating Cyphal DSDL C++ types with nunavut"
    VERBATIM
)

add_custom_target(dsdl_types ALL DEPENDS "${DSDL_TYPES_STAMP}")

# Make generated headers visible to all source files in the project.
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${DSDL_TYPES_DIR}")
add_dependencies(${CMAKE_PROJECT_NAME} dsdl_types)
