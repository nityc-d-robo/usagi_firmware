/**
 * @file actuator_command.h
 * @brief Planar/Bit/Readiness デコードとコマンド状態管理。タイムアウト処理付き。
 */

#ifndef ACTUATOR_COMMAND_H
#define ACTUATOR_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** コマンド状態を初期化し、RX サブスクリプションを CyphalTransport に登録する。 */
void actuator_command_init(void);

/** 現在のコマンド状態をアクチュエータに適用する。タスクループから周期的に呼ぶ。 */
void actuator_command_apply(void);

/** デコードエラー・タイムアウト回数を取得する。 */
void actuator_command_get_stats(uint32_t* decode_errors, uint32_t* timeout_count);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_COMMAND_H */
