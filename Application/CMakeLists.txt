# DSDL embed: generate dsdl_embedded.c from public_regulated_data_types at build time
find_package(Python3 REQUIRED)
set(DSDL_ROOT "${CMAKE_SOURCE_DIR}/Drivers/public_regulated_data_types")
set(DSDL_EMBED_LIST
    reg/udral/physics/dynamics/rotation/Planar.0.1.dsdl
    reg/udral/physics/kinematics/rotation/Planar.0.1.dsdl
    reg/udral/service/common/Readiness.0.1.dsdl
    uavcan/si/unit/torque/Scalar.1.0.dsdl
    uavcan/si/unit/angle/Scalar.1.0.dsdl
    uavcan/si/unit/angular_velocity/Scalar.1.0.dsdl
    uavcan/si/unit/angular_acceleration/Scalar.1.0.dsdl
    uavcan/primitive/scalar/Bit.1.0.dsdl
)
set(DSDL_EMBEDDED_C "${CMAKE_CURRENT_BINARY_DIR}/dsdl_embedded.c")
add_custom_command(OUTPUT "${DSDL_EMBEDDED_C}"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_dsdl.py"
            --dsdl-root "${DSDL_ROOT}"
            --output "${DSDL_EMBEDDED_C}"
            ${DSDL_EMBED_LIST}
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_dsdl.py"
            ${DSDL_ROOT}/reg/udral/physics/dynamics/rotation/Planar.0.1.dsdl
            ${DSDL_ROOT}/reg/udral/physics/kinematics/rotation/Planar.0.1.dsdl
            ${DSDL_ROOT}/reg/udral/service/common/Readiness.0.1.dsdl
            ${DSDL_ROOT}/uavcan/si/unit/torque/Scalar.1.0.dsdl
            ${DSDL_ROOT}/uavcan/si/unit/angle/Scalar.1.0.dsdl
            ${DSDL_ROOT}/uavcan/si/unit/angular_velocity/Scalar.1.0.dsdl
            ${DSDL_ROOT}/uavcan/si/unit/angular_acceleration/Scalar.1.0.dsdl
            ${DSDL_ROOT}/uavcan/primitive/scalar/Bit.1.0.dsdl
    COMMENT "Generating dsdl_embedded.c from DSDL files"
    VERBATIM
)
add_custom_target(dsdl_embedded_c ALL DEPENDS "${DSDL_EMBEDDED_C}")

# Application sources
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/led_blink_node.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_memory.c
    ${DSDL_EMBEDDED_C}
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/dsdl_runtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/cyphal_node.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/actuator_command.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/actuator_output.c
)
add_dependencies(${CMAKE_PROJECT_NAME} dsdl_embedded_c)

# libcanard (Cyphal/CAN transport)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/libcanard/libcanard/canard.c
)

# dsdl.c (runtime DSDL parse/serialize)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/dsdl.c/dsdl.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/tasks
    ${CMAKE_SOURCE_DIR}/Drivers/libcanard/libcanard
    ${CMAKE_SOURCE_DIR}/Drivers/libcanard/lib/cavl2
    ${CMAKE_SOURCE_DIR}/Drivers/dsdl.c
    ${CMAKE_SOURCE_DIR}/Drivers/dsdl.c/lib
)
