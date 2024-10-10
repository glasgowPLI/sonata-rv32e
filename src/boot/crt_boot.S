# Copyright Dejice Jacob
.include "common/assembly-helpers.S"
.section .text.start, "ax", @progbits
.zero 0x80 # Start section with 128-bytes of zeroed data
.global _start
.p2align 2
.type _start,@function
_start:
   .cfi_startproc
   .cfi_undefined ra
   .option push 
   .option norelax
     la gp, __global_pointer$
   .option pop
    /* Enable the ICache */
    #ifndef QEMU_DEBUG
    csrsi  0x7c0, 1
    #endif

    la_abs t0, trap 
    csrw   mtvec, t0

    /* No bounds on bootloader stack, grows down from the end of IRAM */
    la     sp, __stack_top /* __stack_top -> 0x00100ff0 */

    /*!
     * Clear the revocation bitmap before entering C code.
     * The bitmap is not cleared upon reset so memset to return it to a
     * pristine state.
     */
    #if 0  /* Not required for RV32 */
    li     a0,  0x3000000 
    li     a1, 0
    li     a2, 4096
    call   bl_memset
    #endif 
    call   rom_loader_entry
   .cfi_endproc

    /* Flush ICache now we've loaded new code */
    fence.i

    /* Jump to newly loaded binary */
    mv     t0, a0
    jr     t0

.section .text.bl_memset, "ax", @progbits
.global bl_memset
bl_memset:
    /* Check if aligned. If so -- use word fill */
    andi   a3, a0, 3
    bnez   a3, .Lbytes_fill
    andi   a3, a2, 3
    bnez   a3, .Lbytes_fill

    /* Broadcast a1 to all bytes. */
    andi   a1, a1, 0xff 
    slli   a3, a1, 8
    or     a1, a3, a1
    slli   a3, a1, 16
    or     a1, a3, a1

0:
    beqz   a2, .Lret
    sw     a1, (a0)
    addi   a0, a0, 4
    addi   a2, a2, -4
    j      0b

.Lbytes_fill:
0:
    beqz    a2, .Lret
    sb      a1, (a0)
    addi    a0, a0, 1
    addi    a2, a2, -1
    j       0b

.Lret:
    ret

.section .text.trap, "ax", @progbits
// Trap handler must be 4 byte aligned.
.p2align 2
trap:
    li      sp, 0x00100ff0
    call    exception_handler
    // Infinite loop if the exception handler even returns
1:
    wfi
    j 1b
