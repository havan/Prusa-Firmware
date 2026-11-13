//! @file
//! @brief Runtime diagnostics for hunting silent stalls and RAM corruption.
//!
//! Enabled with DIAG_BUILD (see variants/MK3S.h). The checks are cheap and run
//! from manage_heater(), which is the one function every wait loop in the
//! firmware keeps calling (it feeds the watchdog). That means they still run
//! when the main loop is trapped in some nested wait, which is exactly the
//! situation a watchdog-based crash dump can never catch.
//!
//! When a check fires, a non-resetting xflash dump is taken with a dedicated
//! dump_crash_reason, the reason is latched in EEPROM_FW_CRASH_FLAG so the
//! next boot shows a notice, and the event is reported on serial.
//! See DIAG.md in the repository root for the user-facing description.
#pragma once
#include <stdint.h>
#include "Configuration_var.h"

#ifdef DIAG_BUILD

//! Paint the free RAM between .bss and the stack so the stack high-water mark
//! can be measured later. Call once, early in setup().
void diag_init();

//! Record that the print is making progress: a G-code finished, or a
//! legitimately long wait (heating, dwell) is still running.
void diag_progress();

//! Run the periodic checks. Called from manage_heater().
void diag_periodic();

//! D90: print the diagnostics state (and, with DIAG_FAULT_INJECT, inject faults).
void dcode_90();

#else

inline void diag_init() {}
inline void diag_progress() {}
inline void diag_periodic() {}

#endif // DIAG_BUILD
