/**
 ******************************************************************************
 * @file           : platform_hw.h
 * @brief          : Extern declarations for HAL peripheral handles
 *
 * All HAL handles are owned by main.c (CubeIDE-generated). Platform driver
 * files include this header to access them.
 ******************************************************************************
 */

#ifndef PLATFORM_HW_H
#define PLATFORM_HW_H

#include "main.h"

/* --- I2C ----------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;

/* --- SPI ----------------------------------------------------------------- */
extern SPI_HandleTypeDef hspi1;

/* --- UART ---------------------------------------------------------------- */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

/* --- Timers (PWM / DShot) ------------------------------------------------ */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* --- Timers (Scheduler) -------------------------------------------------- */
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

/* --- Timers (RC PPM Input Capture) --------------------------------------- */
extern TIM_HandleTypeDef htim16;

/* --- Platform init called from main after peripherals are ready ---------- */
void platform_hw_uart_start(void);

#endif /* PLATFORM_HW_H */
