/**
  ******************************************************************************
  * @file    Inc/core_selftest.h
  * @brief   OmniNav 算法层（firmware_core）上板自检入口
  *
  *          目的：把 firmware_core 真正链接进固件，并在真机上用"已知正确答案"
  *          断言验证算法层在 Cortex-M3 / armclang 下的行为与宿主测试一致。
  *          断言期望值全部抄自 firmware_core/test/ 下已 25344 条全绿的用例。
  ******************************************************************************
  */

#ifndef CORE_SELFTEST_H
#define CORE_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  跑一遍 firmware_core 五模块（fixedpoint / mecanum / pid / odom / protocol）
  *         的已知答案断言，结果从 USART1 打印
  * @note   只用整数运算；调用前必须已完成 UART 初始化
  */
void core_selftest_run(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_SELFTEST_H */
