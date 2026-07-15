#ifndef LDPC_H
#define LDPC_H

#include <stdint.h>

// CCSDS (8160, 7136) Rate 7/8 LDPC Encoder
// Information block: 7136 bits (892 bytes)
// Parity block: 1024 bits (128 bytes)
// Total Codeword: 8160 bits (1020 bytes)

// Encodes a byte-aligned 892-byte payload into a 1020-byte LDPC codeword
void ldpc_78_encode(const uint8_t *info_bytes, uint16_t info_len, uint8_t *codeword_out);

#endif
