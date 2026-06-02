# hot_shift_smoke

Single-core, no-MMU smoke case for trampoline-based hot shift.

## What this case verifies
1. CPU runs in `hot_shift_migration_wait` loop.
2. TB performs switch flow with `+HOT_SHIFT_SMOKE`.
3. TB injects:
   - `pc = hot_shift_trampoline`
   - `x1(ra) = hot_shift_resume_point`
4. CPU executes trampoline:
   - `fence rw, rw`
   - `fence.i`
   - `sfence.vma x0, x0`
   - `jalr x0, x1, 0`
5. CPU reaches `hot_shift_resume_point` and writes PASS signature `0x444333222`.

## Build
```bash
cd openc910/emu/scripts
make buildcase CASE=hot_shift_smoke
```

## Get trampoline/resume addresses
```bash
cd openc910/emu/scripts/work
riscv64-unknown-elf-nm hot_shift_smoke.elf | grep hot_shift_
```
Expected symbols:
- `hot_shift_trampoline`
- `hot_shift_resume_point`

## Run (iverilog)
```bash
cd openc910/emu/scripts/work
vvp xuantie_core.vvp \
  +HOT_SHIFT_SMOKE \
  +HOT_SHIFT_STUB_PC=<hot_shift_trampoline_addr_hex> \
  +HOT_SHIFT_RESUME_PC=<hot_shift_resume_point_addr_hex> \
  +HOT_SHIFT_CYCLE=5000
```

## Notes
- `HOT_SHIFT_CYCLE` controls when TB triggers switch.
- This smoke uses no MMU setup in case logic; `sfence.vma` is executed in M-mode as part of the trampoline sequence.
