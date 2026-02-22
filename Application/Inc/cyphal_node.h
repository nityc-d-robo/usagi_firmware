/**
 * @file cyphal_node.h
 * @brief Cyphal ノードの lifecycle API と FreeRTOS タスク宣言。
 *
 * main.c (C) から include できるよう extern "C" でラップする。
 * Publish API は cyphal_publish.hpp を参照。
 */

#ifndef CYPHAL_NODE_H
#define CYPHAL_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/** canard / TX キュー / RX キューを初期化する。スケジューラ起動前に呼ぶ。 */
bool cyphal_node_init(void);

/** FreeRTOS タスクエントリ。xTaskCreate に渡す。 */
void CyphalControlTask(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* CYPHAL_NODE_H */
