//! @file
//! @brief Runtime diagnostics for hunting silent stalls and RAM corruption.
//!
//! Background: on a printer without an MMU, a long SD print froze on the MMU
//! screen "MMU Retry: Restoring temperature..." with the head resting on the
//! part. The main loop was trapped in MMU2::ResumeHotendTemp(), a wait loop
//! that keeps feeding the watchdog, so no crash dump was ever written. The
//! power panic data saved afterwards showed a Z position 166 mm off and a
//! hotend target of 256 instead of 240: RAM had been corrupted. The stock
//! firmware has no way to capture the state in such a case. This module adds:
//!
//!  1. A stall detector: an SD print that makes no progress for
//!     DIAG_STALL_TIMEOUT_S seconds gets a dump (diag_stall).
//!  2. A position consistency check: once the planner is empty, the stepper
//!     counters, the planner's target and the logical position must agree.
//!     A disagreement means corrupted motion state; the next move could drive
//!     the nozzle into the bed, so the print is killed after a dump
//!     (diag_position).
//!  3. A stack high-water mark, with a dump when the stack gets close to
//!     .bss (diag_stack). The stock stack guard is a single word and can be
//!     jumped over by a large frame.
//!  4. Reset forensics: a printer that restarts without any dump left no
//!     trace at all. A software restart (a wild jump or return to address 0)
//!     leaves the peripherals configured; a real reset (power, brown-out,
//!     reset pin, watchdog) puts them back to their defaults. The registers are
//!     sampled before the C runtime starts, and the last commands are kept in
//!     .noinit RAM, so the next boot can tell the two apart and show what ran
//!     just before.
//!
//! All checks are rate limited and run from manage_heater(), which every wait
//! loop calls; see diag.h.

#include "diag.h"

#ifdef DIAG_BUILD

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "Marlin.h"
#include "cardreader.h"
#include "cmdqueue.h"
#include "ConfigurationStore.h"
#include "eeprom.h"
#include "lcd.h"
#include "mesh_bed_calibration.h"
#include "planner.h"
#include "sound.h"
#include "stepper.h"
#include "ultralcd.h"
#include "xflash_dump.h"

// Thresholds. Kept as plain constants: changing them is a rebuild anyway.
static constexpr uint16_t DIAG_STALL_TIMEOUT_S = 5 * 60; // no progress for this long = stall
static constexpr uint16_t DIAG_PERIOD_MS = 1000;         // position/stall check period
static constexpr uint8_t DIAG_STACK_PERIOD = 10;         // stack scan every N periods
static constexpr float DIAG_POS_TOLERANCE_MM = 2.0f;     // allowed disagreement (expected: 0)
static constexpr uint16_t DIAG_STACK_MIN_FREE = 128;     // bytes; below this = dump
static constexpr uint8_t DIAG_STACK_PAINT = 0xC5;
static constexpr uint16_t DIAG_REALERT_S = 60;           // beep period while stalled

// Planner/stepper state that has no public declaration.
extern float position_float[NUM_AXIS];
extern char __heap_start; // end of .data + .bss + .noinit

// Reset forensics state. It lives in .noinit, which the C runtime neither
// loads nor clears, so it keeps its content across every reset that doesn't
// cut the power.
static constexpr uint16_t DIAG_CRUMB_MAGIC = 0xD1A9;
static constexpr uint8_t DIAG_CRUMB_CMDS = 4;     // power of two
static constexpr uint8_t DIAG_CRUMB_CMD_LEN = 27; // characters kept per command

struct DiagCrumbs {
    uint16_t magic;
    uint8_t next;       // ring slot the next command goes to
    uint8_t sessions;   // resets since the RAM was last found invalid
    uint32_t uptime_ms; // _millis() at the last periodic check
    struct {
        uint32_t sdpos; // SD read position when the command started
        uint8_t type;   // CMDBUFFER_CURRENT_TYPE_*
        char text[DIAG_CRUMB_CMD_LEN + 1];
    } cmd[DIAG_CRUMB_CMDS];
};
static DiagCrumbs crumbs __attribute__((section(".noinit")));

// Peripheral registers as found at the reset vector, before the Arduino core
// and setup() configure anything. Order: ADCSRA, TIMSK0..TIMSK5.
static uint8_t boot_regs[7] __attribute__((section(".noinit")));

// CPU state at the reset vector, saved by diag_sample_entry() before the C
// runtime resets SP. After a software restart this is the state of the code
// that jumped to 0: Z = 0 points at an icall/eicall through a null pointer,
// and the bytes above SP hold the return addresses of the call chain.
static constexpr uint8_t DIAG_ENTRY_STACK = 128;
extern "C" {
uint8_t diag_entry_regs[32] __attribute__((section(".noinit"), used));
uint8_t diag_entry_io[5] __attribute__((section(".noinit"), used)); // SREG SPL SPH EIND RAMPZ
uint8_t diag_entry_stack[DIAG_ENTRY_STACK] __attribute__((section(".noinit"), used));
}

// Runs in .init1, before .init2 clears r1/SREG and loads SP, so it must not
// touch any register before saving it: pure asm.
extern "C" void diag_sample_entry() __attribute__((naked, used, section(".init1")));
extern "C" void diag_sample_entry() {
    asm volatile(
        ".irp r,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31\n\t"
        "sts diag_entry_regs+\\r, r\\r\n\t"
        ".endr\n\t"
        "in r0, __SREG__\n\t"
        "sts diag_entry_io+0, r0\n\t"
        "in r30, __SP_L__\n\t"
        "in r31, __SP_H__\n\t"
        "sts diag_entry_io+1, r30\n\t"
        "sts diag_entry_io+2, r31\n\t"
        "in r0, 0x3c\n\t" // EIND
        "sts diag_entry_io+3, r0\n\t"
        "in r0, 0x3b\n\t" // RAMPZ
        "sts diag_entry_io+4, r0\n\t"
        // Copy DIAG_ENTRY_STACK bytes from SP+1 (the last byte pushed), clamped
        // so the copy ends at RAMEND.
        "adiw r30, 1\n\t"
        "ldi r24, lo8(%[last])\n\t"
        "ldi r25, hi8(%[last])\n\t"
        "cp r24, r30\n\t"
        "cpc r25, r31\n\t"
        "brsh 1f\n\t"
        "movw r30, r24\n\t"
        "1:\n\t"
        "ldi r26, lo8(diag_entry_stack)\n\t"
        "ldi r27, hi8(diag_entry_stack)\n\t"
        "ldi r24, %[n]\n\t"
        "2:\n\t"
        "ld r0, Z+\n\t"
        "st X+, r0\n\t"
        "dec r24\n\t"
        "brne 2b\n\t"
        :: [last] "i"(RAMEND + 1 - DIAG_ENTRY_STACK), [n] "i"(DIAG_ENTRY_STACK));
}

// Runs in .init3: after __init has set up SP and r1 = 0, before .data and
// .bss are initialised. A naked function placed in an .initN section has no
// prologue, epilogue or ret; execution falls through to the next section.
extern "C" void diag_sample_boot_regs() __attribute__((naked, used, section(".init3")));
extern "C" void diag_sample_boot_regs() {
    boot_regs[0] = ADCSRA;
    boot_regs[1] = TIMSK0;
    boot_regs[2] = TIMSK1;
    boot_regs[3] = TIMSK2;
    boot_regs[4] = TIMSK3;
    boot_regs[5] = TIMSK4;
    boot_regs[6] = TIMSK5;
}

// The firmware keeps the ADC running for the temperature readings from boot
// on. Neither the hardware reset nor the bootloader enables it, so ADEN set at
// the reset vector means the previous session jumped there in software.
static bool warm_restart() {
    return boot_regs[0] & _BV(ADEN);
}

namespace {

enum : uint8_t {
    F_DUMPED_STALL = 1 << 0, // only one dump per reason and boot: the first
    F_DUMPED_POS = 1 << 1,   // event is the interesting one, later ones would
    F_DUMPED_STACK = 1 << 2, // overwrite it
    F_IN_PERIODIC = 1 << 3,  // re-entrancy guard
    F_FORCE_ARMED = 1 << 4,  // D90 S: arm the stall detector without a print
};

uint8_t flags;
uint8_t period_count;
uint16_t stall_timeout_s = DIAG_STALL_TIMEOUT_S;
uint32_t last_progress_ms;
uint32_t last_period_ms;
uint32_t last_alert_ms;
uint16_t stack_min_free = UINT16_MAX;
float max_step_err_mm;  // worst |stepper - planner| seen, for validating the tolerance
float max_logic_err_mm; // worst |planner - world2machine(current_position)| seen

uint8_t *stack_paint_start() {
    // Leave the stock stack guard word (STACK_GUARD_MARGIN, 4 bytes) alone.
    return (uint8_t *)&__heap_start + STACK_GUARD_MARGIN + sizeof(uint32_t);
}

uint16_t stack_free_bytes() {
    const uint8_t *const start = stack_paint_start();
    const uint8_t *p = start;
    const uint8_t *const end = (const uint8_t *)RAMEND;
    while (p < end && *p == DIAG_STACK_PAINT)
        ++p;
    return p - start;
}

void alert() {
    for (uint8_t i = 0; i < 3; ++i) {
        Sound_MakeCustom(150, 0, true);
        _delay(150);
    }
}

//! Take a non-resetting dump with @p reason, at most once per boot and reason,
//! and latch the reason so the next boot tells the user about it.
void take_dump(dump_crash_reason reason, uint8_t once_flag) {
    if (flags & once_flag) {
        puts_P(PSTR("DIAG: dump already taken this boot, not overwriting"));
        return;
    }
    flags |= once_flag;
    const uint32_t t0 = _millis();
    xfdump_dump(reason);
    eeprom_update_byte_notify((uint8_t *)EEPROM_FW_CRASH_FLAG, (uint8_t)reason);
    printf_P(PSTR("DIAG: dump saved (reason %u, %lums)\n"), (unsigned)reason, _millis() - t0);
    puts_P(PSTR("//action:dump_available"));
}

bool stall_detector_armed() {
    if (flags & F_FORCE_ARMED)
        return true;
    return card.sdprinting && !printingIsPaused() && !saved_printing;
}

void check_stall(uint32_t now) {
    // Anything that legitimately keeps the print waiting counts as progress:
    // moves still in the planner, waiting for the user, homing and mesh bed
    // leveling. Heating waits and dwells call diag_progress() themselves.
    if (!stall_detector_armed() || blocks_queued() || homing_flag || mesh_bed_leveling_flag
        || busy_state == PAUSED_FOR_USER || busy_state == PAUSED_FOR_INPUT) {
        last_progress_ms = now;
        return;
    }
    const uint32_t idle_s = (now - last_progress_ms) / 1000;
    if (idle_s < stall_timeout_s)
        return;

    if (!(flags & F_DUMPED_STALL)) {
        printf_P(PSTR("DIAG: stall, no progress for %lus, sdpos %lu, busy_state %d\n"),
                 idle_s, (unsigned long)card.get_sdpos(), (int)busy_state);
        take_dump(dump_crash_reason::diag_stall, F_DUMPED_STALL);
        if (lcd_update_enabled) {
            lcd_setalertstatuspgm(PSTR("DIAG: stall, dumped"));
        } else {
            // A wait loop owns the screen, and what it shows is the best clue
            // to where the firmware is stuck. lcd_setalertstatuspgm() would
            // reinitialize the LCD and wipe it, so only queue the text for the
            // status screen.
            lcd_setstatuspgm(PSTR("DIAG: stall, dumped"));
        }
        last_alert_ms = now;
        alert();
    } else if (now - last_alert_ms >= DIAG_REALERT_S * 1000ul) {
        // Keep reminding: an unattended printer can sit like this for hours.
        last_alert_ms = now;
        alert();
    }
}

void check_position() {
    // Only meaningful while printing with an empty planner and outside the
    // operations that rewrite positions on purpose.
    if (!printJobOngoing() || printingIsPaused() || saved_printing || homing_flag
        || mesh_bed_leveling_flag || planner_aborted || blocks_queued())
        return;

    // The planner keeps X/Y in machine coordinates: plan_buffer_line() and
    // plan_set_position() apply the XYZ calibration (skew and bed offset) to
    // them before storing position_float. current_position is in world
    // coordinates, so transform it the same way before comparing. Z is stored
    // untransformed (mesh leveling only affects the step target).
    float expected[3] = {current_position[X_AXIS], current_position[Y_AXIS],
                         current_position[Z_AXIS]};
    world2machine(expected[X_AXIS], expected[Y_AXIS]);

    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis) {
        const long steps = st_get_position(axis);
        const long planned = position[axis];
        const float step_err = (steps - planned) / cs.axis_steps_per_mm[axis];
        const float logic_err = position_float[axis] - expected[axis];
        const float abs_step_err = fabs(step_err);
        const float abs_logic_err = fabs(logic_err);
        if (abs_step_err > max_step_err_mm)
            max_step_err_mm = abs_step_err;
        if (abs_logic_err > max_logic_err_mm)
            max_logic_err_mm = abs_logic_err;
        if (abs_step_err <= DIAG_POS_TOLERANCE_MM && abs_logic_err <= DIAG_POS_TOLERANCE_MM)
            continue;

        printf_P(PSTR("DIAG: position mismatch axis %c: steps %ld planned %ld "
                      "planner_mm %.3f current_mm %.3f\n"),
                 "XYZ"[axis], steps, planned, (double)position_float[axis],
                 (double)current_position[axis]);
        take_dump(dump_crash_reason::diag_position, F_DUMPED_POS);
        // The motion state is corrupt: the next move may crash the nozzle
        // into the print or the bed. Stop everything.
        kill(PSTR("DIAG: position corrupted. Print stopped, dump saved."));
    }
}

void check_stack() {
    const uint16_t free_bytes = stack_free_bytes();
    if (free_bytes < stack_min_free)
        stack_min_free = free_bytes;
    if (free_bytes < DIAG_STACK_MIN_FREE && !(flags & F_DUMPED_STACK)) {
        printf_P(PSTR("DIAG: stack low, %u bytes left above static RAM\n"), free_bytes);
        take_dump(dump_crash_reason::diag_stack, F_DUMPED_STACK);
        alert();
    }
}

} // anonymous namespace

void diag_init() {
    // Interrupts must be off: an ISR firing now would push its frame below SP,
    // into the area being painted. With them off, only our own frame is live,
    // and the 64 byte margin keeps clear of it. Takes well under 1 ms.
    CRITICAL_SECTION_START;
    uint8_t *p = stack_paint_start();
    uint8_t *const end = (uint8_t *)SP - 64;
    while (p < end)
        *p++ = DIAG_STACK_PAINT;
    CRITICAL_SECTION_END;
    last_progress_ms = _millis();
}

void diag_progress() {
    last_progress_ms = _millis();
}

void diag_periodic() {
    if (flags & F_IN_PERIODIC)
        return;
    const uint32_t now = _millis();
    if (now - last_period_ms < DIAG_PERIOD_MS)
        return;
    last_period_ms = now;
    crumbs.uptime_ms = now;
    flags |= F_IN_PERIODIC;

    check_stall(now);
    check_position();
    if (++period_count >= DIAG_STACK_PERIOD) {
        period_count = 0;
        check_stack();
    }

    flags &= ~F_IN_PERIODIC;
}

void diag_command() {
    auto &c = crumbs.cmd[crumbs.next & (DIAG_CRUMB_CMDS - 1)];
    c.sdpos = card.get_sdpos();
    c.type = CMDBUFFER_CURRENT_TYPE;
    strncpy(c.text, CMDBUFFER_CURRENT_STRING, DIAG_CRUMB_CMD_LEN);
    c.text[DIAG_CRUMB_CMD_LEN] = 0;
    crumbs.next = (crumbs.next + 1) & (DIAG_CRUMB_CMDS - 1);
}

void diag_boot_report() {
    printf_P(PSTR("DIAG: reset %S (ADCSRA %02x TIMSK0-5 %02x %02x %02x %02x %02x %02x)\n"),
             warm_restart() ? PSTR("WARM: software restart, a jump to address 0")
                            : PSTR("cold: power-on, brown-out, reset pin or watchdog"),
             boot_regs[0], boot_regs[1], boot_regs[2], boot_regs[3], boot_regs[4],
             boot_regs[5], boot_regs[6]);
    {
        const uint8_t *r = diag_entry_regs;
        const uint16_t sp = diag_entry_io[1] | (diag_entry_io[2] << 8);
        printf_P(PSTR("DIAG: entry SP %04x SREG %02x EIND %02x RAMPZ %02x X %02x%02x Y %02x%02x Z %02x%02x\n"),
                 sp, diag_entry_io[0], diag_entry_io[3], diag_entry_io[4],
                 r[27], r[26], r[29], r[28], r[31], r[30]);
        if (warm_restart()) {
            // Everything needed to unwind the stack with the ELF: all registers,
            // and the stack bytes from the first one copied (SP+1, clamped).
            printf_P(PSTR("DIAG: entry regs r0..r31 "));
            for (uint8_t i = 0; i < 32; ++i)
                printf_P(PSTR("%02x"), r[i]);
            putchar('\n');
            const uint16_t first = (sp + 1u <= RAMEND + 1u - DIAG_ENTRY_STACK) ? sp + 1u
                                                                              : RAMEND + 1u - DIAG_ENTRY_STACK;
            for (uint8_t i = 0; i < DIAG_ENTRY_STACK; ++i) {
                if (i % 32 == 0)
                    printf_P(PSTR("DIAG: entry stack %04x:"), first + i);
                printf_P(PSTR(" %02x"), diag_entry_stack[i]);
                if (i % 32 == 31)
                    putchar('\n');
            }
        }
    }
    if (crumbs.magic == DIAG_CRUMB_MAGIC && crumbs.next < DIAG_CRUMB_CMDS) {
        ++crumbs.sessions;
        printf_P(PSTR("DIAG: previous session: up %lums, reset #%u since RAM was lost; "
                      "last commands, oldest first:\n"),
                 crumbs.uptime_ms, crumbs.sessions);
        for (uint8_t i = 0; i < DIAG_CRUMB_CMDS; ++i) {
            auto &c = crumbs.cmd[(crumbs.next + i) & (DIAG_CRUMB_CMDS - 1)];
            // The text may be anything if RAM was corrupted: print it safely.
            char text[DIAG_CRUMB_CMD_LEN + 1];
            uint8_t n = 0;
            for (; n < DIAG_CRUMB_CMD_LEN && c.text[n]; ++n)
                text[n] = (c.text[n] >= ' ' && c.text[n] < 0x7f) ? c.text[n] : '?';
            text[n] = 0;
            printf_P(PSTR("DIAG:   type %u sdpos %lu: %s\n"), c.type, c.sdpos, text);
        }
    } else {
        puts_P(PSTR("DIAG: previous session: no breadcrumbs (power was off)"));
        crumbs.sessions = 0;
    }
    // Start a fresh record for this session.
    const uint8_t sessions = crumbs.sessions;
    memset(&crumbs, 0, sizeof(crumbs));
    crumbs.sessions = sessions;
    crumbs.magic = DIAG_CRUMB_MAGIC;
}

/*!
### D90 - Diagnostics status <a href="https://reprap.org/wiki/G-code#D90">D90</a>
Print the state of the DIAG build checks.
#### Usage

    D90 [ T | S | Z | J ]

#### Parameters
- `T` - Set the stall timeout in seconds (0 restores the default). Not saved.
- `S` - (fault injection) Trap the firmware in a wait loop that only feeds the
        watchdog, like the original MMU screen hang, until the knob is pressed.
        The stall detector is armed even without a print. Use with `T`.
- `Z` - (fault injection) Add this many steps to the Z stepper counter. During a
        print the position check must then dump and stop the print.
- `J` - (fault injection) Jump to address 0, like a corrupted return address
        would. The next boot must report a WARM reset and list this D90 as the
        last command.
*/
void dcode_90() {
    if (code_seen('T')) {
        const uint16_t t = code_value_short();
        stall_timeout_s = t ? t : DIAG_STALL_TIMEOUT_S;
    }
#ifdef DIAG_FAULT_INJECT
    if (code_seen('Z')) {
        const long dz = code_value_long();
        CRITICAL_SECTION_START;
        count_position[Z_AXIS] += dz;
        CRITICAL_SECTION_END;
        printf_P(PSTR("DIAG: injected %ld Z steps\n"), dz);
    }
    if (code_seen('J')) {
        puts_P(PSTR("DIAG: jumping to address 0"));
        ((void (*)())0)();
    }
    if (code_seen('S')) {
        puts_P(PSTR("DIAG: stall trap, press the knob to leave"));
        flags |= F_FORCE_ARMED;
        last_progress_ms = _millis();
        lcd_update_enable(false);
        lcd_clear();
        lcd_puts_at_P(0, 1, PSTR("DIAG stall trap"));
        lcd_puts_at_P(0, 2, PSTR("press knob to leave"));
        while (!lcd_clicked())
            delay_keep_alive(100);
        flags &= ~F_FORCE_ARMED;
        lcd_update_enable(true);
        lcd_return_to_status();
    }
#endif // DIAG_FAULT_INJECT
    const uint32_t now = _millis();
    printf_P(PSTR("DIAG: build " FW_VERSION_FULL "_" FW_COMMIT_HASH "\n"
                  "DIAG: stall armed %d, idle %lus, timeout %us\n"
                  "DIAG: stack min free %u bytes (now %u)\n"
                  "DIAG: max position error stepper %.4fmm logical %.4fmm\n"
                  "DIAG: dumps taken this boot 0x%02x\n"
                  "DIAG: last reset %S\n"),
             (int)stall_detector_armed(), (now - last_progress_ms) / 1000, stall_timeout_s,
             stack_min_free, stack_free_bytes(), (double)max_step_err_mm, (double)max_logic_err_mm,
             flags & (F_DUMPED_STALL | F_DUMPED_POS | F_DUMPED_STACK),
             warm_restart() ? PSTR("WARM (software)") : PSTR("cold"));
}

#endif // DIAG_BUILD
