
#pragma once

#include "pins.h"
#include <cstdint>
#include <hardware/clocks.h>
#include <hardware/pio.h>

static const uint16_t oqpsk_program_instr[] = {
    0x7002u,
    0xA042u,
};

static const pio_program_t oqpsk_program_default = {
    .instructions = (uint16_t *)oqpsk_program_instr,
    .length = 2,
    .origin = -1,
};

inline void oqpsk_program_init(PIO pio, uint sm, uint offset, uint32_t clkin_hz) {
  pio_sm_config c = pio_get_default_sm_config();

  sm_config_set_sideset(&c, 1, false, false);
  sm_config_set_sideset_pins(&c, PIN_CLKIN);

  sm_config_set_out_pins(&c, PIN_I_IN, 2);

  sm_config_set_wrap(&c, offset, offset + 1);

  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

  sm_config_set_out_shift(&c, true, true, 32);

  pio_gpio_init(pio, PIN_CLKIN);
  pio_gpio_init(pio, PIN_I_IN);
  pio_gpio_init(pio, PIN_Q_IN);
  pio_sm_set_consecutive_pindirs(pio, sm, PIN_CLKIN, 1, true);
  pio_sm_set_consecutive_pindirs(pio, sm, PIN_I_IN, 2, true);

  uint32_t pio_hz = clkin_hz * 2u;
  float div = (float)clock_get_hz(clk_sys) / (float)pio_hz;
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(pio, sm, offset, &c);
}