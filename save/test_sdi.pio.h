// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------- //
// test_sdi //
// -------- //

#define test_sdi_wrap_target 0
#define test_sdi_wrap 6

static const uint16_t test_sdi_program_instructions[] = {
            //     .wrap_target
    0x98a0, //  0: pull   block           side 3     
    0xf82f, //  1: set    x, 15           side 3     
    0x7001, //  2: out    pins, 1         side 2     
    0x0042, //  3: jmp    x--, 2          side 0     
    0xf03e, //  4: set    x, 30           side 2     
    0x0020, //  5: jmp    !x, 0           side 0     
    0x1045, //  6: jmp    x--, 5          side 2     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program test_sdi_program = {
    .instructions = test_sdi_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config test_sdi_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + test_sdi_wrap_target, offset + test_sdi_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}

static inline void test_sdi_program_init(PIO pio, uint sm, uint offset, uint sdi, uint sideset_base, float div) {
    pio_sm_config c =  test_sdi_program_get_default_config(offset);
    // Set sideset (SCLK/CS)
    sm_config_set_sideset_pins(&c, sideset_base);
    pio_gpio_init(pio, sideset_base);
    pio_gpio_init(pio, sideset_base+1);
    pio_sm_set_consecutive_pindirs(pio, sm, sideset_base, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, sideset_base+1, 1, true);
    // Set out (SDI)
    sm_config_set_out_pins(&c, sdi, 1);
    pio_gpio_init(pio, sdi);
    pio_sm_set_consecutive_pindirs(pio, sm, sdi, 1, true);
    // Set clk div
    sm_config_set_clkdiv(&c, div);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

#endif
