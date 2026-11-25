/**
  ******************************************************************************
  * @file    inc/main.h
  * @brief   Header for main.c module - OmniNav STM32 (ALIENTEK Elite board)
  *
  *  NOTE: Difference from ST's STM3210E_EVAL template: removed
  *      #include "stm3210e_eval.h"
  *    That is the board-level driver for ST's evaluation board
  *    (LCD/SRAM/NOR/NAND/CS43L22...), which the ALIENTEK Elite board does not
  *    have; keeping it would drag a pile of unrelated BSP headers into the build.
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
