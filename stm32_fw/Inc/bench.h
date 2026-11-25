/**
  ******************************************************************************
  * @file    Inc/bench.h
  * @brief   Peripheral-layer bench test (pin probe + single-motor open-loop drive)
  *
  *          Target board: ALIENTEK STM32F103ZE Elite board V2
  *
  *          Three stages:
  *            A heartbeat    -- proves the firmware is running (on-board
  *                              LED0=PB5 / LED1=PE5 toggle)
  *            B pin probe    -- PE7...PE15 / PD0 / PD1 / PD8 / PD9 toggle at
  *                              1 Hz in turn; the UART reports which pin is
  *                              being toggled so that a multimeter on the
  *                              TFT_LCD header can map "MCU pin name <->
  *                              header silkscreen" one by one
  *            C single motor -- PE9 = TIM1_CH1 20 kHz PWM, PE7/PE8 control the
  *                              direction, PE15 = TB6612 STBY; operated with
  *                              single-character UART commands
  *
  *          NOTE: No external module is required; stages A and B run without
  *          any wiring.
  ******************************************************************************
  */

#ifndef __BENCH_H
#define __BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/**
  * @brief  Run all three stages (infinite loop inside, never returns)
  */
void bench_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __BENCH_H */
