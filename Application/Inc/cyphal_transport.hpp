/**
 * @file cyphal_transport.hpp
 * @brief CyphalTransport: canard インスタンス・TX/RX キュー・FDCAN ブリッジを所有する transport 層。
 *
 * publish は push() を呼ぶ。subscribe はコールバックを渡して subscribe() を呼ぶ。
 * FreeRTOS タスクや application 層はこのクラスに依存してよいが、
 * このクラス自体は application 層 (actuator_command 等) を知らない。
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <array>

#include "canard.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "fdcan.h"

class CyphalTransport {
public:
    static constexpr size_t kMaxSubscriptions = 8;

    static CyphalTransport& instance();

    /** canard / TX キュー / RX キューを初期化する。スケジューラ起動前に呼ぶ。 */
    bool init(CanardNodeID node_id = 0);

    /** タスクハンドルを登録する。CyphalControlTask の先頭で呼ぶ。 */
    void set_task_handle(TaskHandle_t handle);

    /** FDCAN 通知を有効化してコントローラを開始する。 */
    void start_fdcan();

    /** RX 処理 + TX フラッシュを 1 回実行する。タスクループから呼ぶ。 */
    void step();

    /** RX キュー溢れカウンタ。 */
    uint32_t frames_dropped() const;

    /**
     * シリアライズ済みペイロードを TX キューに積む。
     * transfer_id はインクリメントされる（呼び出し側が管理）。
     */
    bool push(CanardPortID subject_id, CanardTransferID& transfer_id,
              const uint8_t* payload, size_t size);

    /**
     * RX サブスクリプションを登録する。
     * 対応する転送を受信すると callback が呼ばれる。
     * init() の後、start_fdcan() の前に呼ぶこと。
     */
    bool subscribe(CanardPortID subject_id, size_t extent,
                   std::function<void(const CanardRxTransfer&)> callback);

    /** ISR から呼ぶ。HAL_FDCAN_RxFifo0Callback の実体。 */
    void isr_rx(FDCAN_HandleTypeDef* hfdcan);

private:
    struct RxFrame {
        uint32_t can_id;
        uint8_t  size;
        uint8_t  data[CANARD_MTU_CAN_FD];
    };

    struct Sub {
        CanardRxSubscription                         entry;
        std::function<void(const CanardRxTransfer&)> callback;
    };

    CanardInstance  canard_{};
    CanardTxQueue   tx_queue_{};
    QueueHandle_t   rx_queue_{nullptr};
    TaskHandle_t    task_handle_{nullptr};
    uint32_t        frames_dropped_{0};

    std::array<Sub, kMaxSubscriptions> subs_{};
    size_t sub_count_{0};

    static constexpr uint32_t kRxQueueLen      = 16;
    static constexpr uint32_t kTxQueueCapacity = 64;
    static constexpr uint32_t kTxDeadlineMs    = 100;

    void process_rx();
    void flush_tx();
    static uint8_t dlc_to_len(uint32_t dlc);
};
