/**
  ******************************************************************************
  * @file    Inc/core_selftest.h
  * @brief   On-board self-test entry for the OmniNav algorithm layer (firmware_core)
  *
  *          Goal: link firmware_core into the firmware for real and, on the
  *          target, assert against known-good answers that the algorithm layer
  *          behaves the same under Cortex-M3 / armclang as it does in the host
  *          tests. All expected assertion values are copied from the 25344
  *          all-green cases under firmware_core/test/.
  ******************************************************************************
  */

#ifndef CORE_SELFTEST_H
#define CORE_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Run the known-answer assertions for the five firmware_core modules
  *         (fixedpoint / mecanum / pid / odom / protocol); the result is
  *         printed on USART1
  * @note   Integer arithmetic only; UART must already be initialized
  */
void core_selftest_run(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_SELFTEST_H */
