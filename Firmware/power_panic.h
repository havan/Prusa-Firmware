#pragma once

namespace PowerPanic {
enum PowerPanicFlag : uint8_t {
    NO_PENDING_RECOVERY = 0,
    PENDING_RECOVERY = 1, // First power panic, print state is saved in EEPROM
    PENDING_RECOVERY_RETRY = 2, // Power outage occured during recovery, print is still saved in EEPROM
};

// Types of printjobs possible when power panic is triggered
enum PrintType : uint8_t {
    PRINT_TYPE_SD = 0,
    PRINT_TYPE_HOST = 1,
    PRINT_TYPE_NONE = 2,
};
} // namespace PowerPanic

void uvlo_();
void recover_print(uint8_t automatic);

/// Sanity-check the print state saved in EEPROM before resuming it.
/// The resume cannot home Z: it trusts the saved Z and moves down to the
/// print from there. If that Z is wrong the nozzle is driven into the print
/// or the bed. Returns false when the saved Z disagrees with the Z of the
/// G-code segment that was being executed.
bool uvlo_saved_z_plausible();
void setup_uvlo_interrupt();
