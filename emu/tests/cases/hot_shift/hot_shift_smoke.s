/*
 * hot_shift_smoke.s
 * Single-core, no-MMU smoke case for QEMU->HW hot shift via trampoline.
 *
 * Flow:
 * 1) CPU runs in migration_wait loop.
 * 2) TB forces PC to hot_shift_trampoline and x1(ra)=hot_shift_resume_point.
 * 3) Trampoline executes fence/fence.i/sfence.vma, then jalr to x1.
 * 4) Resume point writes PASS signature 0x444333222.
 */

.text
.align 6
.global main
main:
  .include "core_init.h"

  li x5, 0

.global hot_shift_migration_wait
hot_shift_migration_wait:
  addi x5, x5, 1
  addi x6, x6, 3
  xor  x7, x5, x6
  andi x7, x7, 0x7f
  j hot_shift_migration_wait

.align 4
.global hot_shift_trampoline
hot_shift_trampoline:
  fence rw, rw
  fence.i
  sfence.vma x0, x0
  jalr x0, x1, 0

.align 4
.global hot_shift_resume_point
hot_shift_resume_point:
  li x20, 0x444333222
1:
  j 1b
