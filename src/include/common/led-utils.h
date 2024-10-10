/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "timer-utils.hh"
#include <platform/platform-gpio.hh>

typedef volatile SonataGPIO * &GpioRef;

static constexpr uint32_t spiFlash_LED = 0; 
static constexpr uint32_t ElfLoad_LED = 1; 

class DebugLED
{
  private:
    GpioRef  gpio;
    static constexpr uint32_t numLEDs = 8;

  public:
    DebugLED(GpioRef gpio_) : gpio(gpio_) {}

    void set(uint32_t led, bool turnON)
    {
      if (led > numLEDs)
        return;

      if (turnON) { 
        gpio->led_on(led); 
      } else { 
        gpio->led_off(led); 
      }
    }

    void set_all(bool turnON)
    {
      for (uint32_t led = 0 ; led < numLEDs; led++) 
        this->set(led, turnON);
    }

    void blink(uint32_t led, uint32_t on_ms, uint32_t off_ms, uint32_t cycles)
    {
      if (led > numLEDs)
        return;
      for (uint32_t idx = 0 ; idx < cycles; idx++) {
        this->set(led, true);
        spin_wait_ms(on_ms); 
        this->set(led, false);
        spin_wait_ms(off_ms);
      }
    }

    void walk_leds(uint32_t on_ms, uint32_t off_ms, uint32_t cycles) 
    {
      for (uint32_t led = 0 ; led < numLEDs; led++) 
        this->blink(led, on_ms, off_ms, cycles); 
    }
};
