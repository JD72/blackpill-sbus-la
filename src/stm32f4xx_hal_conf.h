/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf.h
  * @brief   HAL configuration, reduced from stm32f4xx_hal_conf_template.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#if !defined(HSE_VALUE)
#define HSE_VALUE 25000000U
#endif
#define HSE_STARTUP_TIMEOUT 100U

#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE 32000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif
#define LSE_STARTUP_TIMEOUT 5000U
#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE 12288000U
#endif

#define VDD_VALUE 3300U
#define TICK_INT_PRIORITY 0x0FU
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE 1U

#define USE_HAL_FLASH_REGISTER_CALLBACKS 0U
#define USE_HAL_PWR_REGISTER_CALLBACKS   0U
#define USE_HAL_RCC_REGISTER_CALLBACKS   0U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"
#endif

#define assert_param(expr) ((void)0U)

#ifdef __cplusplus
}
#endif

#endif
