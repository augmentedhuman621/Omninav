/**
  ******************************************************************************
  * @file    Inc/bench.h
  * @brief   外设层台架测试（引脚探针 + 单电机开环驱动）
  *
  *          目标板：正点原子 STM32F103ZE 精英板 V2
  *
  *          三段阶梯：
  *            A 心跳     —— 证明固件在跑（板载 LED0=PB5 / LED1=PE5 翻转）
  *            B 引脚探针 —— PE7…PE15 / PD0 / PD1 / PD8 / PD9 依次 1 Hz 翻转，
  *                          串口报出"现在翻转的是哪个脚"，用于在 TFT_LCD 排座上
  *                          用万用表把「MCU 引脚名 ↔ 排座丝印」一一对上
  *            C 单电机   —— PE9 = TIM1_CH1 20 kHz PWM，PE7/PE8 控方向，
  *                          PE15 = TB6612 STBY；用串口单字符命令操作
  *
  *          ★ 全程不依赖任何外部模块，不需要先接线就能跑 A 和 B。
  ******************************************************************************
  */

#ifndef __BENCH_H
#define __BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/**
  * @brief  跑完三段阶梯（内部是死循环，永不返回）
  */
void bench_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __BENCH_H */
