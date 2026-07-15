#pragma once
#include <pico.h>
#include <stdint.h>

// A compact Reed-Solomon (255, 223) implementation for CCSDS.
// Based on a public domain implementation.

static inline uint8_t gf_exp[512]; // Expanded to avoid modulo 255 in math
static inline uint8_t gf_log[256];
static inline uint8_t rs_generator_poly[33];
static inline int rs_generator_poly_log[33];
static inline uint8_t rs_mult_table[32][256]; // Precomputed generator

static uint8_t gf_mul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0)
    return 0;
  return gf_exp[(int)gf_log[a] + (int)gf_log[b]];
}

inline void rs_init() {
  // Generate GF(2^8) log and anti-log tables
  gf_exp[0] = 1;
  gf_log[0] = 0;
  gf_log[1] = 0;
  uint16_t x = 1;
  for (int i = 1; i < 255; i++) {
    x <<= 1;
    if (x & 0x100) {
      x ^= 0x187;
    }
    gf_exp[i] = x;
    gf_log[x] = i;
  }
  gf_exp[255] = gf_exp[0];

  // Fill the rest of the expanded table to avoid bounds checks during addition
  for (int i = 256; i < 512; i++) {
    gf_exp[i] = gf_exp[i - 255];
  }

  // Generate CCSDS RS generator polynomial
  rs_generator_poly[0] = 1;
  for (int i = 1; i <= 32; i++) {
    rs_generator_poly[i] = 0;
  }

  // Match standard CCSDS roots
  for (int i = 0; i < 32; i++) {
    uint8_t root = gf_exp[((112 + i) * 11) % 255];
    for (int j = i + 1; j > 0; j--) {
      rs_generator_poly[j] =
          rs_generator_poly[j - 1] ^ gf_mul(rs_generator_poly[j], root);
    }
    rs_generator_poly[0] = gf_mul(rs_generator_poly[0], root);
  }

  // Precompute logs of the generator polynomial
  for (int i = 0; i <= 32; i++) {
    rs_generator_poly_log[i] =
        (rs_generator_poly[i] == 0) ? -1 : gf_log[rs_generator_poly[i]];
  }

  // Precompute generator polynomial multiplication table
  for (int j = 0; j < 32; j++) {
    uint8_t coeff = rs_generator_poly[31 - j];
    for (int val = 0; val < 256; val++) {
      rs_mult_table[j][val] = gf_mul(coeff, val);
    }
  }
}

inline void __not_in_flash_func(rs_encode)(const uint8_t *msg,
                                           uint8_t *parity) {
  const int K = 223;
  const int PARITY_LEN = 32;

  for (int i = 0; i < PARITY_LEN; i++) {
    parity[i] = 0;
  }

  for (int i = 0; i < K; i++) {
    uint8_t feedback = msg[i] ^ parity[0];
    if (feedback != 0) {
      for (int j = 0; j < PARITY_LEN - 1; j++) {
        parity[j] = parity[j + 1] ^ rs_mult_table[j][feedback];
      }
      parity[PARITY_LEN - 1] = rs_mult_table[PARITY_LEN - 1][feedback];
    } else {
      for (int j = 0; j < PARITY_LEN - 1; j++) {
        parity[j] = parity[j + 1];
      }
      parity[PARITY_LEN - 1] = 0;
    }
  }
}

inline void __not_in_flash_func(rs_encode_interleaved)(const uint8_t *data,
                                                       uint8_t *parity, int I) {
  for (int i = 0; i < I; i++) {
    uint8_t block_data[223];
    uint8_t block_parity[32];
    for (int j = 0; j < 223; j++) {
      block_data[j] = data[j * I + i];
    }
    rs_encode(block_data, block_parity);
    for (int j = 0; j < 32; j++) {
      parity[j * I + i] = block_parity[j];
    }
  }
}
