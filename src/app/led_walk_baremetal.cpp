#include "common/defs.h"
#include "common/led-utils.h"
#include "common/uart-utils.hh"

#include <platform/platform-uart.hh>
#include <platform/platform-gpio.hh>


const char prefix[] = "\x1b[35mbootloader\033[0m: ";
static constexpr uint32_t NumLeds = 8;
typedef volatile OpenTitanUart * &UartRef;

[[noreturn]] void wait_for_interrupt(UartRef uart, const char *str)
{
        write_str(uart, prefix);
        write_str(uart, str);
        while (true) {
                asm("wfi");
        }
} 
extern "C" { 
[[noreturn]] void cpp_entry_point()
{
  volatile SonataGPIO *gpio = reinterpret_cast<volatile SonataGPIO *>(GPIO_ADDRESS);
  volatile OpenTitanUart *uart = reinterpret_cast<volatile OpenTitanUart *>(UART_ADDRESS);
  DebugLED debug_LED(gpio); 

  uart->init(BAUD_RATE);
  write_str(uart, "Look pretty LEDs!\r\n");

  int  count    = 0;
  bool switchOn = true;
  while (true) {
    if (switchOn) {
      gpio->led_on(count);
    } else {
      gpio->led_off(count);
    };
    spin_wait_ms(500);
    switchOn = (count == NumLeds - 1) ? !switchOn : switchOn;
    count    = (count < NumLeds - 1) ? count + 1 : 0;
    if (!switchOn && (count == 0)) {
      debug_LED.set_all(false);
      spin_wait_ms(500);
      debug_LED.set_all(true);
      spin_wait_ms(500);
      debug_LED.set_all(false);
      spin_wait_ms(500);
      debug_LED.set_all(true);
    }
  }

  wait_for_interrupt(uart, "Error: Waiting for interrupt");
}
} 

