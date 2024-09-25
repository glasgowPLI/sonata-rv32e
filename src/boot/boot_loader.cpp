/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stddef.h>
#include <stdint.h>
#include <algorithm>

#include "common/defs.h"
#include "common/flash-utils.hh"
#include "common/led-utils.h"
#include "common/elf.h"
#include "common/uart-utils.hh"

#include <platform/platform-uart.hh>
#include <platform/platform-spi.hh>
#include <platform/platform-gpio.hh>


#ifdef QEMU_DEBUG
#include "data/payload.h"
#endif 

extern "C" {
  // Use a different name to avoid resolve to CHERIoT-RTOS memset symbol.
  void bl_memset(void *, int, size_t);
}

#define DEBUG_ELF_HEADER 0

const char prefix[] = "\x1b[35mbootloader\033[0m: ";

// This is using GNU statement expression extension so we can return a value
// from a macro: https://gcc.gnu.org/onlinedocs/gcc/Statement-Exprs.html
#define READ_CSR(name)                             \
  ({                                               \
    uint32_t result;                               \
    asm volatile("csrr %0, " name : "=r"(result)); \
    result;                                        \
   })



typedef volatile OpenTitanUart * &UartRef;

[[noreturn]] void complain_and_loop(UartRef uart, const char *str)
{
	write_str(uart, prefix);
	write_str(uart, str);
	while (true) {
		asm("wfi");
	}
}

#if 0 
static void debug_print_phdr(UartRef uart, Elf32_Phdr &phdr)
{
	write_str(uart, prefix);
	write_hex(uart, phdr.p_offset);
	write_str(uart, " ");
	write_hex(uart, phdr.p_vaddr);
	write_str(uart, " ");
	write_hex(uart, phdr.p_filesz);
	write_str(uart, " ");
	write_hex(uart, phdr.p_memsz);
	write_str(uart, "\r\n");
}
#endif 

static void write_hex_with_prefix(UartRef uart, const char *msg, uint32_t value)
{
  write_str(uart, prefix);
  write_str(uart, msg);
  write_hex(uart, value);
  write_str(uart, "\r\n");
}


#ifdef QEMU_DEBUG
void *memcpy(uint8_t *dest, uint8_t *src, size_t sz)
{
  uint8_t *origin = dest; 
  while(sz--)
    *dest++ = *src++; 
  return (void *)origin;
}

uint32_t qemu_read_elf(uint8_t *flash,
                  UartRef   uart,
                  uint8_t  *sram,
                  uint8_t  *hyperram)
{
  Elf32_Ehdr ehdr;
  memcpy((uint8_t *)&ehdr, flash, sizeof(Elf32_Ehdr));

  // Check the ELF magic numbers.
  if (ehdr.e_ident[EI_MAG0] != ELFMAG0 || ehdr.e_ident[EI_MAG1] != ELFMAG1 ||
      ehdr.e_ident[EI_MAG2] != ELFMAG2 || ehdr.e_ident[EI_MAG3] != ELFMAG3)
  {
    complain_and_loop(uart, "Failed ELF Magic Check\r\n");
  }

  if (ehdr.e_ident[EI_CLASS] != ELFCLASS32 || ehdr.e_type != ET_EXEC ||
      ehdr.e_machine != EM_RISCV ||
      (ehdr.e_flags & EF_RISCV_E) != EF_RISCV_E)
      /* (ehdr.e_flags & (EF_RISCV_CHERIABI | EF_RISCV_CAP_MODE)) !=
        (EF_RISCV_CHERIABI | EF_RISCV_CAP_MODE)) */
  {
    complain_and_loop(uart,
                      "ELF file is not 32-bit CHERI RISC-V executable\r\n");
  }

#if DEBUG_ELF_HEADER
  write_str(uart, prefix);
  write_str(uart, "Offset VirtAddr FileSize MemSize\r\n");
#endif

  Elf32_Phdr phdr;
  for (uint32_t i = 0; i < ehdr.e_phnum; i++)
  {
    memcpy((uint8_t *)&phdr, 
           flash + ehdr.e_phoff + ehdr.e_phentsize * i,
           sizeof(Elf32_Phdr));

    if (phdr.p_type != PT_LOAD)
      continue;

#if DEBUG_ELF_HEADER
    debug_print_phdr(uart, phdr);
#endif

    uint8_t *segment = phdr.p_vaddr >= QEMU_SRAM_ADDRESS_TOP ? hyperram : sram;
    segment = (uint8_t *)phdr.p_vaddr;
    for (uint32_t offset = 0; offset < phdr.p_filesz; offset += 0x400)
    {
      uint32_t size = std::min(phdr.p_filesz, offset + 0x400) - offset;
      memcpy(segment + offset, flash + phdr.p_offset + offset, size);
    }

    bl_memset(segment + phdr.p_filesz, 0, phdr.p_memsz - phdr.p_filesz);
  }

  return ehdr.e_entry;
}
#endif // #define QEMU_DEBUG


uint32_t read_elf(SpiFlash  &flash,
                  UartRef   uart,
                  uint8_t  *sram,
                  uint8_t  *hyperram)
{
  write_str(uart, prefix);
  write_str(uart, "Loading software from flash...\r\n");

  Elf32_Ehdr ehdr;
  flash.read(0x0, (uint8_t *)&ehdr, sizeof(Elf32_Ehdr));

  // Check the ELF magic numbers.
  if (ehdr.e_ident[EI_MAG0] != ELFMAG0 || ehdr.e_ident[EI_MAG1] != ELFMAG1 ||
      ehdr.e_ident[EI_MAG2] != ELFMAG2 || ehdr.e_ident[EI_MAG3] != ELFMAG3)
  {
    complain_and_loop(uart, "Failed ELF Magic Check\r\n");
  }

  if (ehdr.e_ident[EI_CLASS] != ELFCLASS32 || ehdr.e_type != ET_EXEC ||
      ehdr.e_machine != EM_RISCV ||
      (ehdr.e_flags & EF_RISCV_E) != EF_RISCV_E)
  {
    complain_and_loop(uart, "ELF file is not 32-bit RISCV(e) executable\r\n");
  }

#if DEBUG_ELF_HEADER
  write_str(uart, prefix);
  write_str(uart, "Offset VirtAddr FileSize MemSize\r\n");
#endif

  Elf32_Phdr phdr;
  for (uint32_t i = 0; i < ehdr.e_phnum; i++)
  {
    flash.read(ehdr.e_phoff + ehdr.e_phentsize * i,
               (uint8_t *)&phdr, sizeof(Elf32_Phdr));

    if (phdr.p_type != PT_LOAD)
      continue;

#if DEBUG_ELF_HEADER
    debug_print_phdr(uart, phdr);
#endif

    uint8_t *segment = phdr.p_vaddr >= (SRAM_ADDRESS + SRAM_BOUNDS) ? hyperram : sram;
    segment = (uint8_t *)phdr.p_vaddr;
    #if 0 // not required for CHERI code
    segment.address() = phdr.p_vaddr;
    segment.bounds().set_inexact(phdr.p_memsz);

    if (!segment.is_valid())
    {
      debug_print_phdr(uart, phdr);
      complain_and_loop(uart,
                        "Cannot get a valid capability for segment\n");
    }
    #endif 

    for (uint32_t offset = 0; offset < phdr.p_filesz; offset += 0x400)
    {
      uint32_t size = std::min(phdr.p_filesz, offset + 0x400) - offset;
      flash.read( phdr.p_offset + offset, segment + offset, size);
    }

    bl_memset(segment + phdr.p_filesz, 0, phdr.p_memsz - phdr.p_filesz);
  }

  return ehdr.e_entry;
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */

extern "C" uint32_t rom_loader_entry(void *rwRoot)
{
  void *root = rwRoot;

  // Create a bounded capability to the UART
  #ifdef QEMU_DEBUG
  volatile OpenTitanUart *uart = reinterpret_cast<volatile OpenTitanUart *>(QEMU_UART_ADDRESS);
  volatile SonataSpi *spi = reinterpret_cast<volatile SonataSpi *>(QEMU_SPI_ADDRESS);
  volatile SonataGPIO *gpio = reinterpret_cast<volatile SonataGPIO *>(QEMU_GPIO_ADDRESS);
  uint8_t *sram = reinterpret_cast<uint8_t *>(QEMU_SRAM_ADDRESS + 0x1000);
  uint8_t *hyperram = reinterpret_cast<uint8_t *>(QEMU_HYPERRAM_ADDRESS); 
  #else 
  volatile SonataGPIO *gpio = reinterpret_cast<volatile SonataGPIO *>(GPIO_ADDRESS);
  volatile OpenTitanUart *uart = reinterpret_cast<volatile OpenTitanUart *>(UART_ADDRESS);
  volatile SonataSpi *spi = reinterpret_cast<volatile SonataSpi *>(SPI_ADDRESS);
  uint8_t *sram = reinterpret_cast<uint8_t *>(SRAM_ADDRESS + 0x1000);
  uint8_t *hyperram = reinterpret_cast<uint8_t *>(HYPERRAM_ADDRESS); 
  #endif


  DebugLED debug_LED(gpio); 
  debug_LED.set_all(true); /* Initialise -- sign of life */
  spin_wait_ms(1000);
  debug_LED.set_all(false);  /* Reset for start of debugging */

  spi->init(false, false, true, 0);
  uart->init(BAUD_RATE);


  /* Initialise flash */
  debug_LED.blink(spiFlash_LED, 200 , 200, 3);
  SpiFlash spi_flash(spi, gpio, FLASH_CSN_GPIO_BIT);
  spi_flash.reset();
  debug_LED.set(spiFlash_LED, true);

  debug_LED.blink(ElfLoad_LED, 200 , 200, 3);
  #ifdef QEMU_DEBUG
  uint32_t entrypoint = qemu_read_elf( qemu_debug_payload_elf, uart, sram, hyperram);
  #else 
  uint32_t entrypoint = read_elf(spi_flash, uart, sram, hyperram);
  #endif
  debug_LED.set(ElfLoad_LED, true); 

  write_str(uart, prefix);
  write_str(uart, "Booting into program, hopefully.\r\n");
  debug_LED.set_all(false); 
  return entrypoint;
}


extern "C" void exception_handler(void *rwRoot)
{
  void *root = rwRoot;

  // Create a bounded capability to the UART
  #ifdef QEMU_DEBUG
  volatile OpenTitanUart *uart = reinterpret_cast<volatile OpenTitanUart *>(QEMU_UART_ADDRESS);
  #else 
  volatile OpenTitanUart *uart = reinterpret_cast<volatile OpenTitanUart *>(UART_ADDRESS);
  #endif

  write_str(uart, prefix);
  write_str(uart, "Exception happened during loading.\r\n");

  write_hex_with_prefix(uart, "mepc  : ", READ_CSR("mepc"));
  write_hex_with_prefix(uart, "mcause: ", READ_CSR("mcause"));
  write_hex_with_prefix(uart, "mtval : ", READ_CSR("mtval"));
}
