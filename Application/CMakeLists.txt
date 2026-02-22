# Generate Cyphal DSDL C++ types from public_regulated_data_types using nunavut.
# Replaces the former C generation and embed_dsdl.py / dsdl.c runtime approach.

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_dsdl_types.cmake)

# Application sources
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/led_blink_node.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_memory.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/actuator_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/cyphal_transport.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/cyphal_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/actuator_command.cpp
)

# libcanard (Cyphal/CAN transport)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/libcanard/libcanard/canard.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/tasks
    ${CMAKE_SOURCE_DIR}/Drivers/libcanard/libcanard
    ${CMAKE_SOURCE_DIR}/Drivers/libcanard/lib/cavl2
)
