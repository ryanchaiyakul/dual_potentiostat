// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ----------------------- //
// ads8353_conversion_sclk //
// ----------------------- //

#define ads8353_conversion_sclk_wrap_target 0
#define ads8353_conversion_sclk_wrap 1

static const uint16_t ads8353_conversion_sclk_program_instructions[] = {
            //     .wrap_target
    0x7820, //  0: out    x, 32           side 3     
    0x1842, //  1: jmp    x--, 2          side 3     
            //     .wrap
    0xd800, //  2: irq    nowait 0        side 3     
    0xf05f, //  3: set    y, 31           side 2     
    0x0061, //  4: jmp    !y, 1           side 0     
    0x1084, //  5: jmp    y--, 4          side 2     
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads8353_conversion_sclk_program = {
    .instructions = ads8353_conversion_sclk_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config ads8353_conversion_sclk_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ads8353_conversion_sclk_wrap_target, offset + ads8353_conversion_sclk_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}
#endif

// ---------------------- //
// ads8353_conversion_sdo //
// ---------------------- //

#define ads8353_conversion_sdo_wrap_target 2
#define ads8353_conversion_sdo_wrap 3

static const uint16_t ads8353_conversion_sdo_program_instructions[] = {
    0x3fc0, //  0: wait   1 irq, 0               [31]
    0xe02f, //  1: set    x, 15                      
            //     .wrap_target
    0x4001, //  2: in     pins, 1                    
    0x0040, //  3: jmp    x--, 0                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ads8353_conversion_sdo_program = {
    .instructions = ads8353_conversion_sdo_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config ads8353_conversion_sdo_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ads8353_conversion_sdo_wrap_target, offset + ads8353_conversion_sdo_wrap);
    return c;
}

static inline void ads8353_conversion_sclk_init(PIO pio, 
                                      uint sm, 
                                      uint offset, 
                                      uint sideset_base) {
    pio_sm_config c =  ads8353_conversion_sclk_program_get_default_config(offset);
    // Set sideset (SCLK/CS)
    sm_config_set_sideset_pins(&c, sideset_base);
    pio_gpio_init(pio, sideset_base);
    pio_gpio_init(pio, sideset_base+1);
    pio_sm_set_consecutive_pindirs(pio, sm, sideset_base, 2, true);
    // Set input Shift to LSB
    sm_config_set_out_shift(&c, false, true, 32);
    // Set Hz to 40 MHz
    sm_config_set_clkdiv_int_frac(&c, 3, 32);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
}

static inline void ads8353_conversion_sdo_init(PIO pio, 
                                      uint sm,
                                      uint sm2, 
                                      uint offset, 
                                      uint sdo_a,
                                      uint sdo_b) {
    //sdo a
    pio_sm_config c =  ads8353_conversion_sdo_program_get_default_config(offset);
    // Set input (SDO)
    sm_config_set_in_pins(&c, sdo_a);
    pio_gpio_init(pio, sdo_a);
    pio_sm_set_consecutive_pindirs(pio, sm, sdo_a, 1, false);
    sm_config_set_in_shift(&c, false, true, 16);    // do LSR to preserve bit order
    // Set Hz to 40 MHz
    sm_config_set_clkdiv_int_frac(&c, 3, 32);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    //pio_sm_set_enabled(pio, sm, true);
    // sdo b
    c =  ads8353_conversion_sdo_program_get_default_config(offset);
    // Set input (SDO)
    sm_config_set_in_pins(&c, sdo_b);
    pio_gpio_init(pio, sdo_b);
    pio_sm_set_consecutive_pindirs(pio, sm2, sdo_b, 1, false);
    sm_config_set_in_shift(&c, false, true, 16);   // do LSR to preserve bit order
    // Set Hz to 40 MHz
    sm_config_set_clkdiv_int_frac(&c, 3, 32);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm2, offset, &c);
    //pio_sm_set_enabled(pio, sm2, true);
}

#endif
