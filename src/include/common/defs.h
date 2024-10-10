/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define CPU_TIMER_HZ (30'000'000)
#define BAUD_RATE    (   921'600)

#define SRAM_ADDRESS (0x0010'0000)
#define SRAM_BOUNDS  (0x0004'0000)

#define RGBLED_CTRL_ADDRESS (0x8000'9000)
#define RGBLED_CTRL_BOUNDS  (0x0000'0010)

#define GPIO_ADDRESS (0x8000'0000)
#define GPIO_BOUNDS  (0x0000'0020)

#define UART_BOUNDS   (0x0000'0034)
#define UART_ADDRESS  (0x8010'0000)
#define UART1_ADDRESS (0x8010'1000)

#define SPI_ADDRESS  (0x8030'0000)
#define SPI_BOUNDS   (0x0000'0024)

#define HYPERRAM_ADDRESS (0x4000'0000)
#define HYPERRAM_BOUNDS  (0x0010'0000)

#define FLASH_CSN_GPIO_BIT 12

#ifdef QEMU_DEBUG
#define QEMU_BOOT_LOAD             ((uint32_t)0x80000000)
#define QEMU_BOOT_LOAD_TOP         ((uint32_t)0x80080000)

#define QEMU_GPIO_ADDRESS          ((uint32_t)0x800ff000)
#define QEMU_GPIO_ADDRESS_TOP      ((uint32_t)0x80100000)

#define QEMU_UART_ADDRESS          ((uint32_t)0x10000000)
#define QEMU_UART_ADDRESS_TOP      ((uint32_t)0x10000008)

#define QEMU_SPI_ADDRESS           ((uint32_t)0x80200000)
#define QEMU_SPI_ADDRESS_TOP       ((uint32_t)0x80300000)

#define QEMU_SRAM_ADDRESS          ((uint32_t)0x80300000)
#define QEMU_SRAM_ADDRESS_TOP      ((uint32_t)0x80340000)

#define QEMU_HYPERRAM_ADDRESS      ((uint32_t)0x80340000)
#define QEMU_HYPERRAM_ADDRESS_TOP  ((uint32_t)0x80341000)
#endif 
