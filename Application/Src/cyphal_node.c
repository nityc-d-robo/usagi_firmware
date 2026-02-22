/**
 * @file cyphal_node.c
 * @brief Cyphal node: canard instance, RX queue, FDCAN bridge, TX queue, typed publish API.
 */

#include "cyphal_node.h"
#include "actuator_command.h"
#include "app_memory.h"
#include "fdcan.h"
#include "canard.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include <string.h>

#define TICK_TO_USEC(tick) \
    ((CanardMicrosecond)(tick) * 1000000UL / (CanardMicrosecond)configTICK_RATE_HZ)

#define RX_QUEUE_LEN       16U
#define RX_QUEUE_ITEM_SIZE (sizeof(struct rx_frame))
#define TX_QUEUE_CAPACITY  64U
#define EXTENT_MESSAGE     64U
#define REDUNDANT_IFACE    0
#define TX_DEADLINE_MS     100U   /* frames older than this are dropped from the TX queue */

struct rx_frame {
    uint32_t can_id;
    uint8_t  payload_size;
    uint8_t  data[CANARD_MTU_CAN_FD];
};

static struct CanardInstance s_canard;
static struct CanardTxQueue  s_tx_queue;
static QueueHandle_t         s_rx_queue;
static TaskHandle_t          s_task_handle;
static uint32_t              s_frames_dropped;

/* Per-message-type monotone transfer IDs for outgoing transfers. */
static CanardTransferID s_tid_planar;
static CanardTransferID s_tid_bit;
static CanardTransferID s_tid_readiness;

static struct CanardRxSubscription s_sub_readiness;
static struct CanardRxSubscription s_sub_servo0;
static struct CanardRxSubscription s_sub_servo1;
static struct CanardRxSubscription s_sub_servo2;
static struct CanardRxSubscription s_sub_servo3;
static struct CanardRxSubscription s_sub_pump;

/* ----------------------------------------------------------------------- */
/* FDCAN helpers                                                            */
/* ----------------------------------------------------------------------- */

static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
    static const uint8_t tab[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    if (dlc > 15u) return 0;
    return tab[dlc];
}

/* ----------------------------------------------------------------------- */
/* ISR: FDCAN → RX queue                                                    */
/* ----------------------------------------------------------------------- */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (hfdcan != &hfdcan1 || s_rx_queue == NULL) return;

    FDCAN_RxHeaderTypeDef header;
    struct rx_frame frame;
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) == HAL_OK) {
        frame.can_id       = header.Identifier;
        frame.payload_size = fdcan_dlc_to_len(header.DataLength);
        if (frame.payload_size > CANARD_MTU_CAN_FD) {
            frame.payload_size = CANARD_MTU_CAN_FD;
        }
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

/* ----------------------------------------------------------------------- */
/* RX: drain queue → canardRxAccept → dispatch                             */
/* ----------------------------------------------------------------------- */

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
        const CanardMicrosecond ts = TICK_TO_USEC(xTaskGetTickCount());
        struct CanardRxTransfer        transfer;
        struct CanardRxSubscription*   out_sub = NULL;
        const int8_t result = canardRxAccept(
            &s_canard, ts, &can_frame, REDUNDANT_IFACE, &transfer, &out_sub);

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

/* ----------------------------------------------------------------------- */
/* TX: flush canard TX queue → FDCAN FIFO                                  */
/* ----------------------------------------------------------------------- */

static void flush_tx_queue(void)
{
    const CanardMicrosecond now_usec = TICK_TO_USEC(xTaskGetTickCount());
    for (;;) {
        const struct CanardTxQueueItem* item = canardTxPeek(&s_tx_queue);
        if (item == NULL) {
            break;
        }
        /* Drop timed-out frames. */
        if (item->tx_deadline_usec < now_usec) {
            /* canardTxPop requires a non-const pointer; cast is intentional. */
            canardTxFree(&s_tx_queue, &s_canard,
                         canardTxPop(&s_tx_queue, (struct CanardTxQueueItem*)item));
            continue;
        }
        /* Send via FDCAN.
         * DataLength for STM32 HAL FDCAN uses the raw 4-bit DLC code (0-15).
         * CanardCANLengthToDLC[n] maps byte-length → 4-bit DLC, which matches. */
        FDCAN_TxHeaderTypeDef hdr = {
            .Identifier          = item->frame.extended_can_id,
            .IdType              = FDCAN_EXTENDED_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = (uint32_t)CanardCANLengthToDLC[item->frame.payload.size],
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_ON,
            .FDFormat            = FDCAN_FD_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0,
        };
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr,
                                          (const uint8_t*)item->frame.payload.data) != HAL_OK) {
            /* TX FIFO full; retry on the next step. */
            break;
        }
        canardTxFree(&s_tx_queue, &s_canard,
                     canardTxPop(&s_tx_queue, (struct CanardTxQueueItem*)item));
    }
}

/* ----------------------------------------------------------------------- */
/* Lifecycle                                                                */
/* ----------------------------------------------------------------------- */

bool cyphal_node_init(void)
{
    struct CanardMemoryResource mem = app_memory_canard_resource();
    s_canard   = canardInit(mem);
    s_tx_queue = canardTxInit(TX_QUEUE_CAPACITY, CANARD_MTU_CAN_FD, mem);
    s_canard.node_id = 0; /* anonymous; set via register or hardcode later */

    s_rx_queue = xQueueCreate(RX_QUEUE_LEN, RX_QUEUE_ITEM_SIZE);
    if (s_rx_queue == NULL) {
        return false;
    }
    s_frames_dropped = 0;
    s_tid_planar     = 0;
    s_tid_bit        = 0;
    s_tid_readiness  = 0;

    const size_t          extent  = EXTENT_MESSAGE;
    const CanardMicrosecond timeout = CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC;

    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage,
                          (CanardPortID)CYPHAL_SUBJECT_READINESS,
                          extent, timeout, &s_sub_readiness) < 0) { return false; }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage,
                          (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_0,
                          extent, timeout, &s_sub_servo0) < 0)    { return false; }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage,
                          (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_1,
                          extent, timeout, &s_sub_servo1) < 0)    { return false; }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage,
                          (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_2,
                          extent, timeout, &s_sub_servo2) < 0)    { return false; }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage,
                          (CanardPortID)CYPHAL_SUBJECT_SERVO_SETPOINT_3,
                          extent, timeout, &s_sub_servo3) < 0)    { return false; }
    if (canardRxSubscribe(&s_canard, CanardTransferKindMessage,
                          (CanardPortID)CYPHAL_SUBJECT_PUMP_SETPOINT,
                          extent, timeout, &s_sub_pump) < 0)      { return false; }

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
    flush_tx_queue();
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

/* ----------------------------------------------------------------------- */
/* Typed publish API (internal helper)                                     */
/* ----------------------------------------------------------------------- */

static bool publish_message(CanardPortID subject_id,
                            CanardTransferID* const transfer_id,
                            const uint8_t* payload,
                            size_t payload_size)
{
    const CanardMicrosecond now_usec =
        TICK_TO_USEC(xTaskGetTickCount());
    const CanardMicrosecond deadline_usec =
        now_usec + (CanardMicrosecond)TX_DEADLINE_MS * 1000UL;

    const struct CanardTransferMetadata meta = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = subject_id,
        .remote_node_id = CANARD_NODE_ID_UNSET,
        .transfer_id    = (*transfer_id)++,
    };
    const struct CanardPayload canard_payload = {
        .size = payload_size,
        .data = payload,
    };
    const int32_t result = canardTxPush(
        &s_tx_queue, &s_canard, deadline_usec, &meta, canard_payload, now_usec, NULL);
    return (result >= 0);
}

/* ----------------------------------------------------------------------- */
/* Typed publish API (public)                                               */
/* ----------------------------------------------------------------------- */

bool cyphal_node_publish_planar(CanardPortID subject_id,
                                const reg_udral_physics_dynamics_rotation_Planar_0_1* msg)
{
    if (msg == NULL) { return false; }
    uint8_t buf[reg_udral_physics_dynamics_rotation_Planar_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t  buf_size = sizeof(buf);
    if (reg_udral_physics_dynamics_rotation_Planar_0_1_serialize_(msg, buf, &buf_size) < 0) {
        return false;
    }
    return publish_message(subject_id, &s_tid_planar, buf, buf_size);
}

bool cyphal_node_publish_bit(CanardPortID subject_id,
                             const uavcan_primitive_scalar_Bit_1_0* msg)
{
    if (msg == NULL) { return false; }
    uint8_t buf[uavcan_primitive_scalar_Bit_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t  buf_size = sizeof(buf);
    if (uavcan_primitive_scalar_Bit_1_0_serialize_(msg, buf, &buf_size) < 0) {
        return false;
    }
    return publish_message(subject_id, &s_tid_bit, buf, buf_size);
}

bool cyphal_node_publish_readiness(CanardPortID subject_id,
                                   const reg_udral_service_common_Readiness_0_1* msg)
{
    if (msg == NULL) { return false; }
    uint8_t buf[reg_udral_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t  buf_size = sizeof(buf);
    if (reg_udral_service_common_Readiness_0_1_serialize_(msg, buf, &buf_size) < 0) {
        return false;
    }
    return publish_message(subject_id, &s_tid_readiness, buf, buf_size);
}
