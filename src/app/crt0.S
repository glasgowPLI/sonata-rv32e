.section .init, "ax"
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
  la sp, __stack_top
  addi sp, sp, -0x4
  add s0, sp, zero
  jal zero, cpp_entry_point
  .cfi_endproc
  .end
