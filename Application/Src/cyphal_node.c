/**
 * @file cyphal_node.c
 * @brief Cyphal node: canard instance, RX queue, FDCAN callback, task step.
 */

#include "cyphal_node.h"
#include "actuator_command.h"
#include "app_memory.h"
#include "dsdl_runtime.h"
#include "fdcan.h"
#include "canard.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include <string.h>

#define TICK_TO_USEC(tick)  ((CanardMicrosecond)(tick) * 1000000UL / (CanardMicrosecond)configTICK_RATE_HZ)

#define RX_QUEUE_LEN    16
#define RX_QUEUE_ITEM_SIZE  (sizeof(struct rx_frame))
#define EXTENT_MESSAGE   64
#define REDUNDANT_IFACE  0

struct rx_frame {
    uint32_t can_id;
    uint8_t  payload_size;
    uint8_t  data[CANARD_MTU_CAN_FD];
};

static struct CanardInstance s_canard;
static QueueHandle_t s_rx_queue;
static TaskHandle_t s_task_handle;
static uint32_t s_frames_dropped;

static struct CanardRxSubscription s_sub_readiness;
static struct CanardRxSubscription s_sub_servo0;
static struct CanardRxSubscription s_sub_servo1;
static struct CanardRxSubscription s_sub_servo2;
static struct CanardRxSubscription s_sub_servo3;
static struct CanardRxSubscription s_sub_pump;

static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
    static const uint8_t tab[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    if (dlc > 15u) return 0;
    return tab[dlc];
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (hfdcan != &hfdcan1 || s_rx_queue == NULL) return;

    FDCAN_RxHeaderTypeDef header;
    struct rx_frame frame;
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) == HAL_OK) {
        frame.can_id       = header.Identifier;
        frame.payload_size = fdcan_dlc_to_len(header.DataLength);
        if (frame.payload_size > CANARD_MTU_CAN_FD) frame.payload_size = CANARD_MTU_CAN_FD;

        BaseType_t woken = pdFALSE;
        if (xQueueSendFromISR(s_rx_queue, &frame, &woken) != pdTRUE) {
            s_frames_dropped++;
        }
        if (s_task_handle != NULL) {
            vTaskNotifyGiveFromISR(s_task_handle, &woken);
        }
        portYIELD_FROM_ISR(woken);
    }
}

static void process_rx_queue(void)
{
    struct rx_frame frame;
    while (xQueueReceive(s_rx_queue, &frame, 0) == pdTRUE) {
        struct CanardFrame can_frame = {
            .extended_can_id = frame.can_id,
            .payload = {
                .size = frame.payload_size,
                .data = frame.data,
            },
        };
        CanardMicrosecond ts = TICK_TO_USEC(xTaskGetTickCount());
        struct CanardRxTransfer transfer;
        struct CanardRxSubscription* out_sub = NULL;
        int8_t result = canardRxAccept(
            &s_canard,
            ts,
            &can_frame,
            REDUNDANT_IFACE,
            &transfer,
            &out_sub);

        if (result == 1 && out_sub != NULL) {
            actuator_command_on_transfer(&transfer, out_sub);
            if (transfer.payload.data != NULL && transfer.payload.allocated_size > 0) {
                s_canard.memory.deallocate(
                    s_canard.memory.user_reference,
                    transfer.payload.allocated_size,
                    transfer.payload.data);
            }
        }
    }
}

bool cyphal_node_init(void)
{
    struct CanardMemoryResource mem = app_memory_canard_resource();
    s_canard = canardInit(mem);
    s_canard.node_id = 0; /* anonymous or set later */

    if (!dsdl_runtime_init()) {
        return false;
    }

    s_rx_queue = xQueueCreate(RX_QUEUE_LEN, RX_QUEUE_ITEM_SIZE);
    if (s_rx_queue == NULL) {
        return false;
    }
    s_frames_dropped = 0;

    const size_t extent = EXTENT_MESSAGE;
    const CanardMicrosecond timeout = CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC;

    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage, (CanardPortID)CYPHAL_SUBJECT_READINESS,
                          extent, timeout, &s_sub_readiness) < 0) {
        return false;
    }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage, (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_0,
                          extent, timeout, &s_sub_servo0) < 0) {
        return false;
    }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage, (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_1,
                          extent, timeout, &s_sub_servo1) < 0) {
        return false;
    }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage, (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_2,
                          extent, timeout, &s_sub_servo2) < 0) {
        return false;
    }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage, (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_3,
                          extent, timeout, &s_sub_servo3) < 0) {
        return false;
    }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage, (CanardPortID)CYPHAL_SUBJECT_PUMP_SETPOINT,
                          extent, timeout, &s_sub_pump) < 0) {
        return false;
    }

    return true;
}

void cyphal_node_start_fdcan(void)
{
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return;
    }
}

void cyphal_node_step(void)
{
    process_rx_queue();
    actuator_command_apply();
}

uint32_t cyphal_node_get_frames_dropped(void)
{
    return s_frames_dropped;
}

void CyphalControlTask(void* pvParameters)
{
    (void)pvParameters;
    s_task_handle = xTaskGetCurrentTaskHandle();
    cyphal_node_start_fdcan();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        cyphal_node_step();
    }
}
