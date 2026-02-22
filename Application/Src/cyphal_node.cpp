/**
 * @file cyphal_node.cpp
 * @brief FreeRTOS タスクのみ。transport の step と actuator の apply を回す。
 */

#include "cyphal_transport.hpp"
#include "cyphal_publish.hpp"
#include "actuator_command.h"
#include "cyphal_node.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include <uavcan/node/Heartbeat_1_0.hpp>

extern "C" bool cyphal_node_init(void)
{
    return CyphalTransport::instance().init();
}

extern "C" void CyphalControlTask(void* pvParameters)
{
    (void)pvParameters;
    auto& transport = CyphalTransport::instance();
    transport.set_task_handle(xTaskGetCurrentTaskHandle());
    transport.start_fdcan();

    TickType_t last_heartbeat = xTaskGetTickCount();
    static CanardTransferID tid_heartbeat{0};

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        transport.step();
        actuator_command_apply();

        TickType_t now = xTaskGetTickCount();
        if ((now - last_heartbeat) >= pdMS_TO_TICKS(1000)) {
            last_heartbeat = now;
            uavcan::node::Heartbeat_1_0 hb{};
            hb.uptime = static_cast<std::uint32_t>(now / configTICK_RATE_HZ);
            hb.health.value = uavcan::node::Health_1_0::NOMINAL;
            hb.mode.value   = uavcan::node::Mode_1_0::OPERATIONAL;
            hb.vendor_specific_status_code = 0;

            cyphal::publish(uavcan::node::Heartbeat_1_0::_traits_::FixedPortId, tid_heartbeat, hb);
        }
    }
}
