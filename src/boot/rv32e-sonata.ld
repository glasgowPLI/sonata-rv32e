/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
OUTPUT_FORMAT("elf32-littleriscv", "elf32-littleriscv",
	      "elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY
{
  /* sonata system memory */
  /* RAM (rwx)  : ORIGIN = 0x00100000, LENGTH = 256K */
  RAM (rwx)  : ORIGIN = 0x00100000, LENGTH = 0x1000 
}


SECTIONS
{
    . = ORIGIN(RAM);
    _start = .;
    PROVIDE (__stack_top = ORIGIN(RAM) + LENGTH(RAM) - 0x4);
    .rom_loader _start :
    {
        *(.init)
        . = ALIGN(4);
        *(.text.start)
        . = ALIGN(4);
        *(.text .text.* .rodata .rodata.* .data.rel.ro)
        . = ALIGN(4);
    } 

    .sdata :
    { 
      __SDATA_BEGIN__ = . ;
      *(.sdata .sdata.*)
      . = ALIGN(4);
    }

    .bss :
    {
      *(.dynbss)
      *(.bss .bss.*)
      . = ALIGN(4);
       __BSS_END__ = .;
    }
    __global_pointer$ = MIN(__SDATA_BEGIN__, __BSS_END__ - 0x800);
}

