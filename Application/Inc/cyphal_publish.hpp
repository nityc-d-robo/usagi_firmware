/**
 * @file cyphal_publish.hpp
 * @brief 型付き Cyphal Publish API（汎用テンプレート）。
 *
 * 責務の分離:
 * - 型依存: 各 DSDL 型の serialize 呼び出し・バッファサイズ（nunavut C++ 生成に委譲）。
 * - 型非依存: CyphalTransport::push(subject_id, tid, payload, size) による
 *   TX キュー投入・メタデータ構築・送信境界（transport 層が担当）。
 *
 * 使い方:
 *   #include <uavcan/node/Heartbeat_1_0.hpp>
 *   uavcan::node::Heartbeat_1_0 hb{};
 *   hb.uptime = ...;
 *   static CanardTransferID tid{0};
 *   cyphal::publish::publish(uavcan::node::Heartbeat_1_0::_traits_::FixedPortId, tid, hb);
 */

#pragma once

#include "canard.h"
#include "cyphal_transport.hpp"
#include "nunavut/support/serialization.hpp"
#include <cstddef>

namespace cyphal {

/**
 * nunavut C++ で生成された型をシリアライズして送信する。
 * T は _traits_::SerializationBufferSizeBytes と serialize(obj, bitspan) を持つこと。
 * tid は呼び出し側で保持し、同一 subject でインクリメントされる。
 */
template<typename T>
bool publish(CanardPortID subject_id, CanardTransferID& tid, const T& obj)
{
    constexpr std::size_t N = T::_traits_::SerializationBufferSizeBytes;
    uint8_t buf[N];
    nunavut::support::bitspan span(buf, N, 0U);
    auto result = serialize(obj, span);
    if (!result) return false;
    return CyphalTransport::instance().push(subject_id, tid, buf, result.value());
}

} // namespace cyphal
