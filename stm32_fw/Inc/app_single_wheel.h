/**
  ******************************************************************************
  * @file    Inc/app_single_wheel.h
  * @brief   A-02 最小单轮安全固件（默认禁能 / 输出限幅 / 200ms 超时撤输出）
  *
  *  ★ 编译期模式开关 OMNINAV_APP_SINGLE_WHEEL：
  *      0 = 默认值：main.c 保持既有行为（上电 3s 内按 'm' 进 bench_run()，
  *          否则进 usart2_bench_run()）——已验收基线不变；
  *      1 = main.c 在 core_selftest_run() 之后进入 app_single_wheel_run()，
  *          即本文件对应的单轮安全固件（台架用）。
  *
  *  ★ 本固件不新增任何冻结协议帧：单轮调试入口只走 USART1（电脑调试口，
  *    板载 CH340C），与冻结的 USART2（PA2/PA3，树莓派协议口）物理隔离。
  *
  *  ★ 2026-11 说明：本轮不得运行 Keil / 不得烧录，本文件与
  *    app_single_wheel.c 的“能否编译通过”由交付后的 Keil 构建确认。
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

/** @brief 进入单轮安全固件主循环（死循环，不返回） */
void app_single_wheel_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SINGLE_WHEEL_H */
