#ifndef LDPC_H
#define LDPC_H

#include <stdint.h>

// CCSDS (8160, 7136) Rate 7/8 LDPC Encoder
// Information block: 7136 bits (892 bytes)
// Parity block: 1024 bits (128 bytes)
// Total Codeword: 8160 bits (1020 bytes)

// In-place encode: buf already contains the info bytes at its start (offset 0).
// On return the parity bytes occupy the parity region (offset info_len .. 1019).
void ldpc_78_encode(uint8_t *buf, uint16_t info_len);

#endif
