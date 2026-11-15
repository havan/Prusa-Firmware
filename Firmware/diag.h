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

//! Record the command that is about to run. The last few are kept in RAM that
//! survives a reset, so the next boot can show what ran just before it.
void diag_command();

//! Report how the previous session ended: a software restart (a jump to the
//! reset vector, which leaves the peripherals configured) or a hardware reset,
//! plus the last commands it ran. Call once, after the boot banner.
void diag_boot_report();

//! Lowest and highest raw 24 V supply reading (VOLT_PWR_PIN, oversampled) since boot.
extern uint16_t diag_vpwr_min, diag_vpwr_max;

//! Track the supply voltage range. Called from the ADC interrupt with each new reading.
inline void diag_voltage_sample(uint16_t raw) {
    if (raw < diag_vpwr_min)
        diag_vpwr_min = raw;
    if (raw > diag_vpwr_max)
        diag_vpwr_max = raw;
}

//! Raw 1.1 V bandgap readings (sum of ADC_OVRSAMPL samples against AVCC) since
//! boot: the MCU's 5 V supply is 1.1 V * 1024 * ADC_OVRSAMPL / reading. A high
//! reading means a low supply. Updated after every ADC cycle (adc.cpp).
extern uint16_t diag_vbg_min, diag_vbg_max, diag_vbg_last;

inline void diag_bandgap_sample(uint16_t raw) {
    diag_vbg_last = raw;
    if (raw < diag_vbg_min)
        diag_vbg_min = raw;
    if (raw > diag_vbg_max)
        diag_vbg_max = raw;
}

#else

inline void diag_init() {}
inline void diag_progress() {}
inline void diag_periodic() {}
inline void diag_command() {}
inline void diag_boot_report() {}
inline void diag_voltage_sample(uint16_t) {}
inline void diag_bandgap_sample(uint16_t) {}

#endif // DIAG_BUILD
