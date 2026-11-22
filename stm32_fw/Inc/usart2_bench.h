/**
 * usart2_bench.h —— USART2 链路自证台（PA2 / PA3）
 * -----------------------------------------------------------------------------
 * 唯一导出：usart2_bench_run()，内部死循环，永不返回。
 * 上电默认入口（在 main() 的 core_selftest_run() 之后调用）。
 *
 * 详细说明、接线表、坑清单见 Src/usart2_bench.c 的文件头注释。
 */

#ifndef USART2_BENCH_H
#define USART2_BENCH_H

/**
 * 跑 USART2 链路自证。内部 for(;;)，永不返回。
 * 前置：main() 已经跑过 HAL_Init / SystemClock_Config / MX_GPIO_Init /
 *       MX_USART1_UART_Init / timebase_repair。
 */
void usart2_bench_run(void);

#endif /* USART2_BENCH_H */
