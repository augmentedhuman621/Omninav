/**
 * usart2_bench.h -- USART2 link self-test bench (PA2 / PA3)
 * -----------------------------------------------------------------------------
 * Sole export: usart2_bench_run(), an infinite loop that never returns.
 * Default power-on entry (called after core_selftest_run() in main()).
 *
 * See the file-header comment in Src/usart2_bench.c for details, the wiring
 * table and the pitfall list.
 */

#ifndef USART2_BENCH_H
#define USART2_BENCH_H

/**
 * Run the USART2 link self-test. Infinite for(;;) inside, never returns.
 * Preconditions: main() has already run HAL_Init / SystemClock_Config /
 *       MX_GPIO_Init / MX_USART1_UART_Init / timebase_repair.
 */
void usart2_bench_run(void);

#endif /* USART2_BENCH_H */
