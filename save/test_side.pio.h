// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --------- //
// test_side //
// --------- //

#define test_side_wrap_target 0
#define test_side_wrap 3

static const uint16_t test_side_program_instructions[] = {
            //     .wrap_target
    0x99a0, //  0: pull   block           side 3 [1] 
    0xb027, //  1: mov    x, osr          side 2     
    0x0020, //  2: jmp    !x, 0           side 0     
    0x1042, //  3: jmp    x--, 2          side 2     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program test_side_program = {
    .instructions = test_side_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config test_side_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + test_side_wrap_target, offset + test_side_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}

static inline void test_side_program_init(PIO pio, uint sm, uint offset, uint sideset_base, float div) {
    pio_sm_config c =  test_side_program_get_default_config(offset);
    // Set sideset (SCLK/CS)
    sm_config_set_sideset_pins(&c, sideset_base);
    // Set clk div
    sm_config_set_clkdiv(&c, div);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, sideset_base);
    pio_gpio_init(pio, sideset_base+1);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, sideset_base, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, sideset_base+1, 1, true);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

#endif
