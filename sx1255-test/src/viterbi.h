#ifndef VITERBI_H
#define VITERBI_H

#include <stdint.h>

// CCSDS convolutional encoder polynomials (K=7, rate 1/2)
// G1 = 1111001b = 171 octal
// G2 = 1011011b = 133 octal
static constexpr uint8_t CONV_G1 = 0x79;
static constexpr uint8_t CONV_G2 = 0x5B;

// Compute XOR parity of all bits in a byte
constexpr uint8_t bpsk_parity8(uint8_t value) {
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

struct BPSKConvFastLut {
    uint16_t byte_out[256];
    uint16_t state_out[64];
    uint8_t next_state[256];

    constexpr BPSKConvFastLut() : byte_out{}, state_out{}, next_state{} {
        for (int b = 0; b < 256; b++) {
            uint16_t out_word = 0;
            uint8_t c = 0;
            for (int bit = 7; bit >= 0; bit--) {
                uint8_t input_bit = (b >> bit) & 0x01;
                c = (uint8_t)(((c >> 1) | (input_bit << 6)) & 0x7F);
                out_word = (out_word << 2) | ((bpsk_parity8(c & CONV_G1) << 1) |
                                              (bpsk_parity8(c & CONV_G2)));
            }
            byte_out[b] = out_word;
            next_state[b] = c >> 1;
        }

        for (int s = 0; s < 64; s++) {
            uint16_t out_word = 0;
            uint8_t c = s << 1;
            for (int bit = 7; bit >= 0; bit--) {
                c = (uint8_t)((c >> 1) & 0x7F);
                out_word = (out_word << 2) |
                           ((bpsk_parity8(c & CONV_G1) << 1) | bpsk_parity8(c & CONV_G2));
            }
            state_out[s] = out_word;
        }
    }
};

// Compile-time generated lookup table instance
static constexpr BPSKConvFastLut CCSDS_CONV_FAST_LUT;

static constexpr uint8_t PUNC_PERIOD[] = {1, 2, 3, 5, 7};
static constexpr uint8_t PUNC_C1[] = {0b1, 0b01, 0b101, 0b10101, 0b1010001};
static constexpr uint8_t PUNC_C2[] = {0b1, 0b11, 0b011, 0b01011, 0b0101111};

struct alignas(4) BPSKPunctureFastLut {
  // table[rate][phase][byte] = (next_phase << 16) | (count << 8) | bits
  uint32_t table[5][7][256];

  constexpr BPSKPunctureFastLut() : table{} {
    for (int rate = 1; rate <= 4; rate++) {
      uint8_t p_period = PUNC_PERIOD[rate];
      uint8_t c1_mask = PUNC_C1[rate];
      uint8_t c2_mask = PUNC_C2[rate];

      for (int phase = 0; phase < p_period; phase++) {
        for (int val = 0; val < 256; val++) {
          uint8_t accum = 0;
          uint8_t count = 0;
          uint8_t current_phase = phase;

          for (int s = 3; s >= 0; s--) {
            uint8_t c1 = (val >> (s * 2 + 1)) & 1;
            uint8_t c2 = (val >> (s * 2)) & 1;
            if ((c1_mask >> current_phase) & 1) {
              accum = (accum << 1) | c1;
              count++;
            }
            if ((c2_mask >> current_phase) & 1) {
              accum = (accum << 1) | c2;
              count++;
            }

            current_phase++;
            if (current_phase >= p_period)
              current_phase = 0;
          }
          table[rate][phase][val] = accum | (count << 8) | (current_phase << 16);
        }
      }
    }
  }
};

static constexpr BPSKPunctureFastLut CCSDS_PUNC_FAST_LUT;

#endif // VITERBI_H
