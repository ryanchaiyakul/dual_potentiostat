// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------------------ //
// ads8353_read_write //
// ------------------ //

#define ads8353_read_write_wrap_target 0
#define ads8353_read_write_wrap 7

static const uint16_t ads8353_read_write_program_instructions[] = {
            //     .wrap_target
    0x98a0, //  0: pull   block           side 3     
    0xfa2f, //  1: set    x, 15           side 3 [2] 
    0x7101, //  2: out    pins, 1         side 2 [1] 
    0x4001, //  3: in     pins, 1         side 0     
    0x0042, //  4: jmp    x--, 2          side 0     
    0xf13e, //  5: set    x, 30           side 2 [1] 
    0x0120, //  6: jmp    !x, 0           side 0 [1] 
    0x1146, //  7: jmp    x--, 6          side 2 [1] 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads8353_read_write_program = {
    .instructions = ads8353_read_write_program_instructions,
    .length = 8,
    .origin = -1,
};

static inline pio_sm_config ads8353_read_write_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ads8353_read_write_wrap_target, offset + ads8353_read_write_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}

static inline pio_sm_config ads8353_read_write_init(PIO pio, 
                                                    uint sm, 
                                                    uint offset, 
                                                    uint sdi, 
                                                    uint sdo_a,
                                                    uint sideset_base) {
    pio_sm_config c =  ads8353_read_write_program_get_default_config(offset);
    // Set sideset (SCLK/CS)
    sm_config_set_sideset_pins(&c, sideset_base);
    pio_gpio_init(pio, sideset_base);
    pio_gpio_init(pio, sideset_base+1);
    pio_sm_set_consecutive_pindirs(pio, sm, sideset_base, 2, true);
    // Set out (SDI)
    sm_config_set_out_pins(&c, sdi, 1);
    pio_gpio_init(pio, sdi);
    pio_sm_set_consecutive_pindirs(pio, sm, sdi, 1, true);
    sm_config_set_out_shift(&c, true, false, 16);
    // Set input (SDO)
    sm_config_set_in_pins(&c, sdo_a);
    pio_gpio_init(pio, sdo_a);
    pio_sm_set_consecutive_pindirs(pio, sm, sdo_a, 1, false);
    // Set Hz to 80 MHz
    sm_config_set_clkdiv_int_frac(&c, 3, 144);
    // Set up SDO shift
    sm_config_set_in_shift(&c, false, true, 16);
    return c;
}

#endif
