#pragma once

#include <cstdint>

// CCSDS AOS Level 0 Transfer Frame (VCDU) constants - CCSDS 732.0-B-5
static constexpr uint16_t ASM_SIZE =
    4; // 4-byte Attached Synchronization Marker (not RS encoded)
static constexpr uint16_t VCDU_HEADER_SIZE =
    6; // 6-byte VCDU header (inside RS block)
static constexpr uint16_t MPDU_HEADER_SIZE =
    2; // 2-byte MPDU header (inside RS block)
static constexpr uint16_t VCDU_OFFSET =
    ASM_SIZE; // VCDU starts at byte 4 (inside RS block)
static constexpr uint16_t MPDU_OFFSET =
    VCDU_OFFSET + VCDU_HEADER_SIZE; // MPDU at byte 10 (inside RS block)
static constexpr uint16_t PAYLOAD_OFFSET =
    MPDU_OFFSET + MPDU_HEADER_SIZE; // Payload at byte 12 (inside RS block)

static constexpr uint8_t VCDU_TRANSFER_FRAME_VERSION =
    0x01;                                             // AOS version (2 bits)
static constexpr uint16_t VCDU_SPACECRAFT_ID = 0x000; // All zeros (10 bits)
static constexpr uint8_t VCDU_REPLAY_FLAG = 0;        // Always 0 (1 bit)
static constexpr uint8_t VCDU_CYCLE_USE_FLAG = 1;     // Cycle use flag (1 bit)
// (removed: per-VC cycle now maintained in frame_counter_cycle[], see A-21)
static constexpr uint8_t VCDU_DEFAULT_VCID = 0x00; // Default VCID = 0 (6 bits)
static constexpr uint8_t VCDU_NUM_VC = 64;         // 6-bit VCID spans 64 VCs

// CCSDS Multiplexing Protocol Data Unit (MPDU) Header - CCSDS 732.0-B-5
static constexpr uint8_t MPDU_SEQ_CONTINUATION = 0x00;
static constexpr uint8_t MPDU_SEQ_END = 0x01;
static constexpr uint8_t MPDU_SEQ_START = 0x02;
static constexpr uint8_t MPDU_SEQ_UNSEGMENTED = 0x03;
static constexpr uint16_t MPDU_NO_START_PACKET =
    0xFFFF; // FHP all ones: no packet start
static constexpr uint16_t MPDU_IDLE_DATA = 0xFFFE; // Only idle data

static constexpr int CADU_ASM_SIZE = ASM_SIZE;
static constexpr uint32_t CADU_ASM =
    0x1ACFFC1Du; // CCSDS standard: 0x1A 0xCF 0xFC 0x1D
