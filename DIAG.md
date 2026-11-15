# DIAG build: Prusa i3 MK3S firmware 3.14.1 with runtime diagnostics

This branch (`diag/3.14.1`) is the official **3.14.1** firmware for the Original Prusa
i3 MK3S/MK3S+ with a few small additions. They capture evidence when a print **stalls
silently** or **RAM gets corrupted**, which the stock firmware can't do. It isn't an
official Prusa build.

## Why

A long SD print froze on the MMU screen *"MMU Retry: Restoring temperature..."*, on a
printer that has **no MMU**:

- The nozzle stayed on the part with the heaters on.
- The knob did nothing.
- No crash dump was written: the firmware was stuck in a wait loop that keeps feeding the
  watchdog.

The power-panic data saved afterwards showed corrupted RAM:
- the Z position was 170.7 mm instead of 4.7 mm;
- the hotend target was 256 °C instead of 240 °C.

The power-panic resume then trusted that Z and drove the nozzle into the print.

## What it adds

| | What | When it fires | What happens |
|---|---|---|---|
| 1 | **Stall detector** | An SD print makes no progress for 5 minutes. Heating, moves, dwells, homing, mesh leveling and waiting for the user count as progress. | Memory dump, serial report, LCD status, three beeps, repeated every minute |
| 2 | **Position check** | With no moves queued, the stepper counters, the planner target and the logical position (after the XYZ-calibration transform) disagree by more than 2 mm on X, Y or Z. They should match exactly. | Memory dump, then the print is **stopped** (the next move could crash the nozzle) |
| 3 | **Stack high-water mark** | Less than 128 bytes of free stack are left above the static variables | Memory dump, serial report, beeps |
| 4 | **Safer power-panic resume** | The saved Z differs by more than 5 mm from the Z of the G-code segment that was running | No automatic resume: a warning, then the usual prompt (default **No**) |
| 5 | **Reset forensics** | Every boot | One line saying whether the last reset was a **software restart** (a jump to address 0: the peripherals were still configured) or a **hardware reset** (power-on, brown-out, reset pin, watchdog). Then the last 4 commands before the reset, with their SD file positions, unless the power was off. |
| 6 | **Motion stall report** | During a print, the move at the head of the planner queue doesn't finish for 3 s plus its own nominal duration (the head stops dead while the firmware keeps running) | Serial report: the planner queue and block, the stepper interrupt state (enabled? running?), the waiting command, temperatures. A second line when motion resumes. No dump, the print continues. |
| 7 | **Hardware sanity** | Every boot; continuously for the supply | A march C- RAM test of everything outside `.noinit` (~60 ms), a CRC-16 of the program flash (~0.4 s, compare it with the hex), and the lowest/highest 24 V supply reading (`D90`). A failing part shows up here, not as a firmware fault. |
| 8 | **Float sanity** | A temperature conversion returns inf or NaN (impossible for a table lookup), or a non-finite value reaches the thermal model or the bed PID D-term | Serial report with the inputs. A bad temperature is converted again with interrupts off: a different result means the first computation was disturbed, and the second result is used so one bad value doesn't latch the filters. Seen as a `TM: error \|-inf\|` on an idle printer. |
| 9 | **Idle RAM test** | Continuously while nothing prints or moves | The free stack area, where nested interrupts save registers, is march-tested 8 bytes at a time with interrupts off and restored. A failure is reported once, with beeps. |

- The checks run from `manage_heater()`, so they still run while the main loop is trapped
  in a wait loop.
- Each kind of dump is taken **at most once per boot**, so later events can't overwrite
  the first one.
- After a diagnostic dump, the next boot shows *"DIAG: a problem was detected and a dump
  was saved"*. Read the dump out before the next event overwrites it.

Code: `Firmware/diag.cpp`, `Firmware/diag.h`, and small hooks in `Marlin_main.cpp`,
`temperature.cpp`, `stepper.cpp`, `power_panic.cpp` and `xflash_dump.*`. The version stays 3.14.1
(PrusaSlicer G-code checks keep working). The commit hash in the boot log and M115 tells
this build apart. The EEPROM layout is unchanged, so calibration and settings survive
flashing in both directions.

## D90

Send over USB (e.g. Pronterface, or `printcore`). **Opening the serial port resets the
printer**, so connect *before* starting a print you want to talk to.

| Command | Effect |
|---|---|
| `D90` | Print state: stall timer, stack minimum, largest position error seen, dumps taken, motion stalls, 24 V range, boot RAM test, non-finite float counts, idle RAM test |
| `D90 T<s>` | Set the stall timeout to `<s>` seconds (`T0` = default 300 s). Not saved. |
| `D90 S` | *Fault injection.* Trap the firmware in a watchdog-feeding wait loop, like the original hang, until the knob is pressed. Arms the stall detector even without a print. |
| `D90 Z<steps>` | *Fault injection.* Offset the Z stepper counter by `<steps>` (400 steps = 1 mm). During a print, the position check must dump and stop the print. |
| `D90 J` | *Fault injection.* Jump to address 0, like a corrupted return address. The printer restarts; the boot log must say `reset WARM` and list `D90 J` as the last command. |
| `D90 M<ms>` | *Fault injection.* Switch the stepper interrupt off for `<ms>` ms. Queue a long move first: the motion stall report must fire (TIMSK1 without bit 1, 0 isr/s) and the resume line must follow. |

## Testing it on the printer

1. Flash the `.hex` with PrusaSlicer (*Configuration → Flash printer firmware*). Check
   that the boot log or *Support* shows the new hash.
2. **Stall detector:** connect, then send `D90 T30` and `D90 S`. Within about 30 s it should
   print `DIAG: stall ...` and `DIAG: dump saved (reason 6 ...)`, and beep. Press the knob
   to leave. `D21` reads the dump.
3. **Position check:**
   - Connect, then start a short throwaway print from the SD card.
   - Once it is printing, send `D90 Z2000` (+5 mm), then `M400`. `M400` drains the move
     queue, and the check only runs while the queue is empty, as in a stall.
   - It should report `DIAG: position mismatch axis Z`, save a dump (reason 7) and stop
     with *"DIAG: position corrupted"*.
   - The injected offset only changes the counter, not the motion, so nothing crashes.
4. **Reset forensics:** send `D90 J`. The printer restarts and the boot log must show
   `DIAG: reset WARM` with `D90 J` as the last command. Then close and reopen the serial
   port (this resets through the reset pin): the boot must show `reset cold` and still
   list the last commands.
5. **Normal prints:** after a few real prints, send `D90`. *max position error* should stay
   tiny (≤ 0.05 mm), well under the 2 mm threshold. *stack min free* shows the real stack
   headroom.

## Reading a dump

```sh
# capture: boot log + M115 + D21 (press the knob if the crash screen is up)
uv run prusa_crash_dump.py
# decode, with the ELF of THIS build
python3 tools/dump2bin crash_<ts>.log crash_<ts>_sram.bin
python3 tools/elf_mem_map <elf> crash_<ts>.log > crash_<ts>_vars.txt
```

Dump reasons: 0 manual, 1 stack_error, 2 watchdog, 3 bad_isr, 4/5 bad pull-up ISR, **6
diag_stall, 7 diag_position, 8 diag_stack**.

## Building

Same as the official 3.14.1 (see the upstream README). Pass the version explicitly so the
build reports itself as 3.14.1 with this branch's hash:

```sh
cmake -S . -B build-diag -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/AvrGcc.cmake -DFW_VARIANTS=MK3S \
  -DFW_COMMIT_DSC=v3.14.1-8237 -DFW_COMMIT_HASH=$(git rev-parse --short=9 HEAD) \
  -DPROJECT_VERSION_TIMESTAMP=$(git log -1 --format=%ct)
ninja -C build-diag MK3S_MULTILANG
```

Keep the ELF (`build-diag/build_gen/MK3S/MK3S_lang_base`) of every build you flash: dumps
can only be decoded with the exact ELF.

## Rolling back

Flash the official `MK3S_MK3S+_FW_3.14.1_MULTILANG.hex`. Nothing else is needed.
