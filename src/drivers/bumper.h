/**
 * @file bumper.h
 * @brief 四路低有效防撞条的输入、历史锁存和人工故障复位条件。
 *
 * ISR记录事件，主循环正常每10ms调用bumper_update()；碰撞历史不会随释放
 * 自动消失。应用层负责根据这些位制动，只有显式故障复位路径才尝试清除。
 */
#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>

#define BUMPER_FRONT 0x01U /**< 前防撞条，PC15低有效。 */
#define BUMPER_REAR  0x02U /**< 后防撞条，PC13低有效。 */
#define BUMPER_LEFT  0x04U /**< 左防撞条，PC14低有效。 */
#define BUMPER_RIGHT 0x08U /**< 右防撞条，PB9低有效。 */

/**
 * @brief 上电配置GPIO/EXINT并采样初始按下状态，调用一次。
 * 初始全释放直接满足释放门；初始按下立即锁存对应方向。
 */
void bumper_init(void);
/**
 * @brief 外层EXINT ISR确认并清标志后调用，记录采到的低电平事件。
 * @param changed_mask 本次触发的方向位集合，使用BUMPER_*按位或。
 */
void bumper_edge_irq(uint8_t changed_mask);
/**
 * @brief 每个正常10ms主循环调用，合并事件并累计十个释放周期。
 * 含很短的关中断取数区；不等待100ms、不操作电机、不自动清历史碰撞位。
 */
void bumper_update(void);
/**
 * @brief 读取累计的碰撞方向位，不消耗事件。
 * @return BUMPER_*位掩码；0表示自上次清除后没有已处理的按下事件。
 */
uint8_t bumper_latched(void);
/**
 * @brief 查询实时输入和释放计数是否允许复位。
 * @return 1为当前全释放且计数达门限，0为尚不允许；不屏蔽随后出现的新边沿。
 */
uint8_t bumper_can_clear(void);
/**
 * @brief 供显式FAULT_RESET路径清除历史碰撞位，函数本身不恢复运动。
 * @return 1为检查通过并清除，0为未满足释放条件；不清除ISR待处理邮箱。
 */
uint8_t bumper_clear(void);

#endif
