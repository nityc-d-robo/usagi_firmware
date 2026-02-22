/**
 * @file actuator_command.cpp
 * @brief Planar/Bit/Readiness デコード・コマンド状態管理・タイムアウト処理。
 *
 * init() で CyphalTransport にラムダを登録し、受信時にデコードを実行する。
 * C++ DSDL 生成型と deserialize を使用。
 */

#include "actuator_command.h"
#include "actuator_output.h"
#include "cyphal_transport.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <reg/udral/physics/dynamics/rotation/Planar_0_1.hpp>
#include <reg/udral/service/common/Readiness_0_1.hpp>
#include <uavcan/primitive/scalar/Bit_1_0.hpp>
#include <cmath>

/* Subject IDs (RX) */
static constexpr CanardPortID kSubjectReadiness   = 3005U;
static constexpr CanardPortID kSubjectServo0      = 3010U;
static constexpr CanardPortID kSubjectServo1      = 3011U;
static constexpr CanardPortID kSubjectServo2      = 3012U;
static constexpr CanardPortID kSubjectServo3      = 3013U;
static constexpr CanardPortID kSubjectPump        = 3020U;
static constexpr size_t       kExtent             = 64U;
static constexpr uint32_t     kControlTimeoutMs   = 1000U;

/* コマンド状態 */
static float      s_servo[4];
static bool       s_pump_on;
static uint8_t    s_readiness;
static TickType_t s_last_cmd_tick;
static uint32_t   s_decode_errors;
static uint32_t   s_timeout_count;
static bool       s_in_timeout;

static void apply_safe_state()
{
    s_readiness = 0;
    s_pump_on   = false;
    for (int i = 0; i < 4; i++) s_servo[i] = 0.0f;
    actuator_output_apply(s_servo, s_pump_on, s_readiness);
}

static void decode_planar(uint8_t idx, const uint8_t* payload, size_t size)
{
    if (idx >= 4) return;
    reg::udral::physics::dynamics::rotation::Planar_0_1 msg{};
    nunavut::support::const_bitspan span(payload, size, 0U);
    if (!deserialize(msg, span)) {
        s_decode_errors++;
        return;
    }
    float sp = 0.0f;
    const float pos = msg.kinematics.angular_position.radian;
    const float vel = msg.kinematics.angular_velocity.radian_per_second;
    if (std::isfinite(pos))      sp = pos;
    else if (std::isfinite(vel)) sp = vel;
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    s_servo[idx] = sp;
}

static void decode_bit(const uint8_t* payload, size_t size)
{
    if (size < 1) return;
    uavcan::primitive::scalar::Bit_1_0 msg{};
    nunavut::support::const_bitspan span(payload, size, 0U);
    if (!deserialize(msg, span)) {
        s_decode_errors++;
        return;
    }
    s_pump_on = msg.value;
}

static void decode_readiness(const uint8_t* payload, size_t size)
{
    if (size < 1) return;
    reg::udral::service::common::Readiness_0_1 msg{};
    nunavut::support::const_bitspan span(payload, size, 0U);
    if (!deserialize(msg, span)) {
        s_decode_errors++;
        return;
    }
    s_readiness = msg.value & 3u;
}

extern "C" void actuator_command_init(void)
{
    s_pump_on       = false;
    s_readiness     = 0;
    s_last_cmd_tick = 0;
    s_decode_errors = 0;
    s_timeout_count = 0;
    s_in_timeout    = false;
    for (int i = 0; i < 4; i++) s_servo[i] = 0.0f;
    actuator_output_init();
    apply_safe_state();

    auto& t = CyphalTransport::instance();

    t.subscribe(kSubjectReadiness, kExtent, [](const CanardRxTransfer& tr) {
        s_last_cmd_tick = xTaskGetTickCount();
        decode_readiness(static_cast<const uint8_t*>(tr.payload.data), tr.payload.size);
    });
    t.subscribe(kSubjectServo0, kExtent, [](const CanardRxTransfer& tr) {
        s_last_cmd_tick = xTaskGetTickCount();
        decode_planar(0, static_cast<const uint8_t*>(tr.payload.data), tr.payload.size);
    });
    t.subscribe(kSubjectServo1, kExtent, [](const CanardRxTransfer& tr) {
        s_last_cmd_tick = xTaskGetTickCount();
        decode_planar(1, static_cast<const uint8_t*>(tr.payload.data), tr.payload.size);
    });
    t.subscribe(kSubjectServo2, kExtent, [](const CanardRxTransfer& tr) {
        s_last_cmd_tick = xTaskGetTickCount();
        decode_planar(2, static_cast<const uint8_t*>(tr.payload.data), tr.payload.size);
    });
    t.subscribe(kSubjectServo3, kExtent, [](const CanardRxTransfer& tr) {
        s_last_cmd_tick = xTaskGetTickCount();
        decode_planar(3, static_cast<const uint8_t*>(tr.payload.data), tr.payload.size);
    });
    t.subscribe(kSubjectPump, kExtent, [](const CanardRxTransfer& tr) {
        s_last_cmd_tick = xTaskGetTickCount();
        decode_bit(static_cast<const uint8_t*>(tr.payload.data), tr.payload.size);
    });
}

extern "C" void actuator_command_apply(void)
{
    const TickType_t now = xTaskGetTickCount();
    if ((now - s_last_cmd_tick) > pdMS_TO_TICKS(kControlTimeoutMs)) {
        if (!s_in_timeout) {
            s_timeout_count++;
            s_in_timeout = true;
        }
        apply_safe_state();
    } else {
        s_in_timeout = false;
        actuator_output_apply(s_servo, s_pump_on, s_readiness);
    }
}

extern "C" void actuator_command_get_stats(uint32_t* decode_errors, uint32_t* timeout_count)
{
    if (decode_errors != nullptr) *decode_errors = s_decode_errors;
    if (timeout_count != nullptr) *timeout_count = s_timeout_count;
}
