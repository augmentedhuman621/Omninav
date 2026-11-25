/**
  ******************************************************************************
  * @file    Inc/app_single_wheel.h
  * @brief   A-02 minimal single-wheel safety firmware (disabled by default /
  *          output limited / output dropped after a 200 ms timeout)
  *
  *  NOTE: Compile-time mode switch OMNINAV_APP_SINGLE_WHEEL:
  *      0 = default: main.c keeps the existing behavior (press 'm' within 3 s
  *          of power-on to enter bench_run(), otherwise enter
  *          usart2_bench_run()) -- the accepted baseline is unchanged;
  *      1 = main.c enters app_single_wheel_run() after core_selftest_run(),
  *          i.e. the single-wheel safety firmware implemented in this file
  *          (for bench use).
  *
  *  NOTE: This firmware adds no frozen protocol frame: the single-wheel debug
  *    entry only uses USART1 (the PC debug port, on-board CH340C), physically
  *    isolated from the frozen USART2 (PA2/PA3, the Raspberry Pi protocol port).
  *
  *  NOTE 2026-11: Keil must not be run and the board must not be flashed in
  *    this round; whether this file and app_single_wheel.c compile is confirmed
  *    by the Keil build after delivery.
  ******************************************************************************
  */
#ifndef __APP_SINGLE_WHEEL_H
#define __APP_SINGLE_WHEEL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OMNINAV_APP_SINGLE_WHEEL
#define OMNINAV_APP_SINGLE_WHEEL 0
#endif

/** @brief Enter the single-wheel safety firmware main loop (infinite loop, never returns) */
void app_single_wheel_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SINGLE_WHEEL_H */
