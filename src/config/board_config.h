/** @file board_config.h @brief 板级标定、控制周期及线上ID；运动保护由上位机负责。 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
#define BOARD_CONTROL_PERIOD_MS             10U
#define BOARD_SPEED_PERIOD_MS               20U
#define BOARD_STATUS_PERIOD_MS             100U
#define BOARD_ADC_PERIOD_MS                 20U
/* 仅决定遥测有效性，不触发停车或拒绝运动命令。 */
#define BOARD_IMU_STALE_MS                  50U
#define BOARD_ADC_REPORT_STALE_MS           60U
#define BOARD_WHEEL_CIRCUMFERENCE_MM    267.035f
#define BOARD_TRACK_WIDTH_MM            500.0f
#define BOARD_ENCODER_COUNTS_PER_REV   2640.0f
#define BOARD_PID_KP                         4.0f
#define BOARD_PID_KI                         8.0f
#define BOARD_PID_KD                         0.0f
#define BOARD_PID_LIMIT                    250.0f
#define BOARD_FF_STATIC_PWM                100.0f
#define BOARD_FF_PER_RPM                     9.4f
#define BOARD_RPM_FILTER_ALPHA               0.20f
#define BOARD_LEFT_TARGET_SIGN              -1.0f
#define BOARD_RIGHT_TARGET_SIGN              1.0f
#define BOARD_LEFT_FEEDBACK_SIGN             1.0f
#define BOARD_RIGHT_FEEDBACK_SIGN           -1.0f
/* 原板级采样系数保留，输出单位为mV和mA。 */
#define BOARD_BUS_UV_PER_COUNT           8850UL
#define BOARD_BUS_OFFSET_MV                  0L
#define BOARD_CURRENT_UA_PER_COUNT        806UL
#define BOARD_ADC_ZERO_SAMPLES              20U
#define BOARD_ADC_ZERO_MAX_RAW              19U
#define BOARD_CAN_ID_CONTROL             0x201U
#define BOARD_CAN_ID_STATUS              0x181U
#define BOARD_CAN_ID_ODOMETRY            0x381U
#define BOARD_CAN_ID_REFERENCE           0x401U
#define BOARD_CAN_REFERENCE_SIZE           16U
#endif
