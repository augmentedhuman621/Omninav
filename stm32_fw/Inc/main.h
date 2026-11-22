/**
  ******************************************************************************
  * @file    inc/main.h
  * @brief   Header for main.c module — OmniNav STM32（正点原子精英板）
  *
  *  ★ 与 ST 官方 STM3210E_EVAL 模板的差异：删掉了
  *      #include "stm3210e_eval.h"
  *    那是 ST 评估板的板级驱动（LCD/SRAM/NOR/NAND/CS43L22…），
  *    正点原子精英板上没有，留着会把一堆无关 BSP 头拖进编译。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
