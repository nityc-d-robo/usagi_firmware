/**
 * @file cyphal_transport.cpp
 * @brief CyphalTransport 実装: canard / FDCAN ブリッジ / RX・TX キュー管理。
 */

#include "cyphal_transport.hpp"
#include "app_memory.h"
#include "FreeRTOSConfig.h"
#include <cstring>

#define TICK_TO_USEC(tick) \
    ((CanardMicrosecond)(tick) * 1000000UL / (CanardMicrosecond)configTICK_RATE_HZ)

/* ----------------------------------------------------------------------- */
/* Singleton                                                                */
/* ----------------------------------------------------------------------- */

static CyphalTransport s_instance;

CyphalTransport& CyphalTransport::instance()
{
    return s_instance;
}

/* ----------------------------------------------------------------------- */
/* ISR: HAL callback → isr_rx()                                            */
/* ----------------------------------------------------------------------- */

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    CyphalTransport::instance().isr_rx(hfdcan);
}

void CyphalTransport::isr_rx(FDCAN_HandleTypeDef* hfdcan)
{
    if (hfdcan != &hfdcan1 || rx_queue_ == nullptr) return;

    FDCAN_RxHeaderTypeDef header;
    RxFrame frame;
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) == HAL_OK) {
        frame.can_id = header.Identifier;
        frame.size   = dlc_to_len(header.DataLength);
        if (frame.size > CANARD_MTU_CAN_FD) frame.size = CANARD_MTU_CAN_FD;

        BaseType_t woken = pdFALSE;
        if (xQueueSendFromISR(rx_queue_, &frame, &woken) != pdTRUE) {
            frames_dropped_++;
        }
        if (task_handle_ != nullptr) {
            vTaskNotifyGiveFromISR(task_handle_, &woken);
        }
        portYIELD_FROM_ISR(woken);
    }
}

/* ----------------------------------------------------------------------- */
/* Lifecycle                                                                */
/* ----------------------------------------------------------------------- */

bool CyphalTransport::init(CanardNodeID node_id)
{
    struct CanardMemoryResource mem = app_memory_canard_resource();
    canard_   = canardInit(mem);
    tx_queue_ = canardTxInit(kTxQueueCapacity, CANARD_MTU_CAN_FD, mem);
    canard_.node_id = node_id;

    rx_queue_ = xQueueCreate(kRxQueueLen, sizeof(RxFrame));
    if (rx_queue_ == nullptr) return false;

    frames_dropped_ = 0;
    sub_count_      = 0;
    return true;
}

void CyphalTransport::set_task_handle(TaskHandle_t handle)
{
    task_handle_ = handle;
}

void CyphalTransport::start_fdcan()
{
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return;
    }
    HAL_FDCAN_Start(&hfdcan1);
}

void CyphalTransport::step()
{
    process_rx();
    flush_tx();
}

uint32_t CyphalTransport::frames_dropped() const
{
    return frames_dropped_;
}

/* ----------------------------------------------------------------------- */
/* Subscribe                                                                */
/* ----------------------------------------------------------------------- */

bool CyphalTransport::subscribe(CanardPortID subject_id, size_t extent,
                                std::function<void(const CanardRxTransfer&)> callback)
{
    if (sub_count_ >= kMaxSubscriptions) return false;

    Sub& s = subs_[sub_count_];
    s.callback = std::move(callback);

    const int8_t result = canardRxSubscribe(
        &canard_, CanardTransferKindMessage, subject_id, extent,
        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &s.entry);
    if (result < 0) return false;

    ++sub_count_;
    return true;
}

/* ----------------------------------------------------------------------- */
/* RX: drain queue → canardRxAccept → callback                             */
/* ----------------------------------------------------------------------- */

void CyphalTransport::process_rx()
{
    RxFrame frame;
    while (xQueueReceive(rx_queue_, &frame, 0) == pdTRUE) {
        CanardFrame can_frame = {
            .extended_can_id = frame.can_id,
            .payload = { .size = frame.size, .data = frame.data },
        };
        const CanardMicrosecond ts = TICK_TO_USEC(xTaskGetTickCount());
        CanardRxTransfer        transfer;
        CanardRxSubscription*   out_sub = nullptr;
        const int8_t result = canardRxAccept(
            &canard_, ts, &can_frame, 0, &transfer, &out_sub);

        if (result == 1 && out_sub != nullptr) {
            for (size_t i = 0; i < sub_count_; ++i) {
                if (&subs_[i].entry == out_sub) {
                    subs_[i].callback(transfer);
                    break;
                }
            }
            if (transfer.payload.data != nullptr && transfer.payload.allocated_size > 0) {
                canard_.memory.deallocate(canard_.memory.user_reference,
                                          transfer.payload.allocated_size,
                                          transfer.payload.data);
            }
        }
    }
}

/* ----------------------------------------------------------------------- */
/* TX: flush canard TX queue → FDCAN FIFO                                  */
/* ----------------------------------------------------------------------- */

void CyphalTransport::flush_tx()
{
    const CanardMicrosecond now_usec = TICK_TO_USEC(xTaskGetTickCount());
    for (;;) {
        const CanardTxQueueItem* item = canardTxPeek(&tx_queue_);
        if (item == nullptr) break;

        if (item->tx_deadline_usec < now_usec) {
            canardTxFree(&tx_queue_, &canard_,
                         canardTxPop(&tx_queue_, const_cast<CanardTxQueueItem*>(item)));
            continue;
        }

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
                static_cast<const uint8_t*>(item->frame.payload.data)) != HAL_OK) {
            break;  /* TX FIFO 満杯; 次の step で再試行 */
        }
        canardTxFree(&tx_queue_, &canard_,
                     canardTxPop(&tx_queue_, const_cast<CanardTxQueueItem*>(item)));
    }
}

/* ----------------------------------------------------------------------- */
/* Push (型なし; cyphal_publish.cpp から使う)                              */
/* ----------------------------------------------------------------------- */

bool CyphalTransport::push(CanardPortID subject_id, CanardTransferID& transfer_id,
                           const uint8_t* payload, size_t size)
{
    const CanardMicrosecond now_usec     = TICK_TO_USEC(xTaskGetTickCount());
    const CanardMicrosecond deadline_usec = now_usec + (CanardMicrosecond)kTxDeadlineMs * 1000UL;

    const CanardTransferMetadata meta = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = subject_id,
        .remote_node_id = CANARD_NODE_ID_UNSET,
        .transfer_id    = transfer_id++,
    };
    const CanardPayload canard_payload = { .size = size, .data = payload };
    const int32_t result = canardTxPush(
        &tx_queue_, &canard_, deadline_usec, &meta, canard_payload, now_usec, nullptr);
    return (result >= 0);
}

/* ----------------------------------------------------------------------- */
/* Helper                                                                   */
/* ----------------------------------------------------------------------- */

uint8_t CyphalTransport::dlc_to_len(uint32_t dlc)
{
    static const uint8_t tab[] = { 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64 };
    if (dlc > 15u) return 0;
    return tab[dlc];
}
