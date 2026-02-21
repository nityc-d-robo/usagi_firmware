target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/tasks/led_blink.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/tasks/servo_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/tasks/canopen_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/tasks/motor_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/OD/OD.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/OD
)
