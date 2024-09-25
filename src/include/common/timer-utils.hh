/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdint.h>

static inline uint32_t get_mcycle(void)
{
	uint32_t result;
	asm volatile("csrr %0, mcycle;" : "=r"(result));
	return result;
}

static inline void reset_mcycle(void)
{
	asm volatile("csrw mcycle, x0");
}

static inline void wait_mcycle(uint32_t value)
{
	reset_mcycle();
	while (get_mcycle() < value) {}
}



#define READ_CSR64(csr)                       \
  ({                                          \
    uint64_t val;                             \
    uint32_t high, low;                       \
    __asm __volatile("1: "                    \
                     "csrr t0, " #csr "h\n"   \
                     "csrr %0, " #csr "\n"    \
                     "csrr %1, " #csr "h\n"   \
                     "bne t0, %1, 1b"         \
                     : "=r"(low), "=r"(high)  \
                     :                        \
                     : "t0");                 \
    val = (low | ((uint64_t)high << 32));     \
    val;                                      \
    })

/**
 * Read the cycle counter.  Returns the number of cycles since boot as a 64-bit
 * value.
 */
static inline uint64_t rdcycle64()
{
  return READ_CSR64(mcycle);
}

/** (Code adapted from CHERI-RTOS function thread_millisecond_wait()) 
 * spin-loop for the specified number of milliseconds.
 * Returns the number of milliseconds that the thread actually waited.
 */
static inline uint64_t spin_wait_ms(uint32_t milliseconds)
{
  static const uint32_t CyclesPerMillisecond = CPU_TIMER_HZ / 1'000;
  uint32_t cycles  = milliseconds * CyclesPerMillisecond;
  uint64_t start   = rdcycle64();
  uint64_t end     = start + cycles;
  uint64_t current = start;

  // Spin until we get to the end
  while (current < end) {
    current = rdcycle64();
  }
  current = rdcycle64();
  return (current - start) / CyclesPerMillisecond;
}


