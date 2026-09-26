#include <Arduino.h>
#include <EEPROM.h>
#include <cmath>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/spi.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include "pico/multicore.h"

#include "ccsds_frame.h"
#include "ldpc.h"
#include "oqpsk_pio.h"
#include "pins.h"
#include "randomizer.h"
#include "rs.h"
#include "sx1255.h"
#include "viterbi.h"

#define POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT true

#define SP_FIFO_SIZE 131072
#define TX_FIFO_SIZE 131072u
#define TX_FIFO_MASK (TX_FIFO_SIZE - 1u)
alignas(4) volatile uint8_t sp_fifo[SP_FIFO_SIZE]
    __attribute__((section(".uninitialized")));
static volatile uint32_t sp_fifo_head = 0;
static volatile uint32_t sp_fifo_tail = 0;
static inline uint32_t get_sp_fifo_count() {
  return (sp_fifo_head - sp_fifo_tail) & (SP_FIFO_SIZE - 1);
}
static inline uint32_t get_sp_fifo_free() {
  return SP_FIFO_SIZE - 1 - get_sp_fifo_count();
}
static inline void push_sp_byte(uint8_t b) {
  uint32_t head = sp_fifo_head;
  sp_fifo[head] = b;
  sp_fifo_head = (head + 1) & (SP_FIFO_SIZE - 1);
}
static inline uint8_t pop_sp_byte() {
  uint8_t val = sp_fifo[sp_fifo_tail];
  sp_fifo_tail = (sp_fifo_tail + 1) & (SP_FIFO_SIZE - 1);
  return val;
}
static inline uint8_t peek_sp_byte(uint32_t offset) {
  return sp_fifo[(sp_fifo_tail + offset) & (SP_FIFO_SIZE - 1)];
}
static inline void clear_sp_fifo() {
  sp_fifo_head = 0;
  sp_fifo_tail = 0;
}

static uint16_t current_sp_offset = 0;
static uint16_t current_sp_size = 0;
static bool is_filler = false;
static uint16_t filler_seq = 0;
static uint8_t filler_header[6];

static volatile uint32_t tx_user_frames_generated = 0;
static volatile uint32_t tx_dma_underflows = 0;
static volatile uint32_t perf_bb_us = 0;

#define INPUT_BUFFER_SIZE 64
static char input_buffer[INPUT_BUFFER_SIZE];
static uint16_t input_idx = 0;
static bool waiting_for_input = false;
static char input_mode = '\0';
static bool binary_upload_mode = false;
static unsigned long last_binary_rx_ms = 0;

// ============================================================
// Configuration
// ============================================================

// Symbol rate
static uint32_t symbol_rate_hz = 100000u;

// Target CLKIN frequency. The SX1255 FIR-DAC reconstruction filter runs off
// the internal 32 MHz XOSC, not CLK_IN — its bandwidth is ~600 kHz SSB at
// 24 taps, adequate for all supported symbol rates. CLK_IN here is used only
// for the digital I/Q interface and PIO timing.
static constexpr uint32_t CLKIN_TARGET_HZ = 8000000u;

// Each symbol occupies exactly cycles_per_symbol PIO clock cycles (= CLK_IN
// cycles) We dynamically calculate this to maximize oversampling without
// exceeding SX1255 limits (32 MHz clock)
static uint32_t cycles_per_symbol = 32;
static uint32_t clkin_hz = 0; // Calculated dynamically

// DSB Filter setting (0 to 31). Formula from datasheet:
// BW_3dB = 17.15 / (41 - tx_filter_bw) MHz DSB.
// 0 = 418 kHz DSB / 209 kHz SSB,
// 15 = 660 kHz DSB / 330 kHz SSB,
// 31 = 1715 kHz DSB / 858 kHz SSB (maximum).
static uint8_t filter_bw_val = 0;
// PLL bandwidth — selected dynamically based on symbol rate
static uint8_t current_pll_bw = TXFE3_PLL_BW_75kHz;

// Apply symbol rate and recompute derived timing parameters
static void apply_symbol_rate(uint32_t rate_hz) {
  symbol_rate_hz = rate_hz;
  cycles_per_symbol = CLKIN_TARGET_HZ / rate_hz;
  if (cycles_per_symbol > 512) cycles_per_symbol = 512;
  cycles_per_symbol &= ~1u;
  if (cycles_per_symbol < 2) cycles_per_symbol = 2;
  clkin_hz = symbol_rate_hz * cycles_per_symbol;
}

// QPSK: Q is aligned with I (0 delay cycles)
static constexpr uint32_t Q_OFFSET_CYCLES = 0u;

// PIO / SM
static spi_inst_t *spi = spi1;
static PIO pio = pio0;
static uint sm = 0;

// DMA Configuration for high-performance streaming
// Size of each Ping-Pong DMA buffer (in 32-bit words).
static constexpr uint32_t DMA_BUF_SIZE = 8192;
alignas(4) static uint32_t dma_buf[2][DMA_BUF_SIZE];
static int dma_chan0 = -1;
static int dma_chan1 = -1;
static int next_buf_to_fill = 0;

static SX1255 sx;

// ============================================================
// CCSDS CADU frame transmitter
// ============================================================

// Frame payload size: VCDU(6) + MPDU(2) + payload = FRAME_CODED_BYTES
// For exactly 1020 bytes of encoded data (8160 bits):
// Uncoded Transfer Frame size = 1020 bytes.
// Payload bytes = 1020 - 6 (VCDU) - 2 (MPDU) = 1012 bytes.
static constexpr uint16_t FRAME_PAYLOAD_BYTES = 1012u;
static constexpr uint16_t FRAME_CODED_BYTES =
    VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + FRAME_PAYLOAD_BYTES; // 1020
static constexpr uint16_t FRAME_PARITY_BYTES = 128u;  // RS(255,223) or LDPC parity
static constexpr uint16_t FRAME_PAYLOAD_LIMIT = 892u;  // FRAME_CODED_BYTES - FRAME_PARITY_BYTES
static constexpr uint16_t FULL_PACKET_SIZE = 1024u;    // FRAME_CODED_BYTES + ASM_SIZE (4)
static constexpr uint32_t FRAME_UNCODED_BITS = (uint32_t)FRAME_CODED_BYTES * 8u;
static constexpr uint32_t FRAME_ENC_BITS = FRAME_UNCODED_BITS * 2u; // rate-1/2

// Max space-packet length bound = FIFO capacity minus one (reject only impossible values).
static const uint32_t MAX_SP_PACKET_LEN = SP_FIFO_SIZE - 1;

// Convolutional encoder state (persistent across frames for continuous stream)
static uint8_t conv_state = 0u;

// Per-VC 24-bit frame counters (64 virtual channels, 6-bit VCID). Each VC keeps
// its own count; idle frames on VC 63 do not advance the user VC. (CCSDS
// 732.0-B-5 §4.1.2.4.2)
static uint32_t frame_counter[VCDU_NUM_VC] = {0u};
// Per-VC 4-bit Frame Count Cycle; incremented when a VC's 24-bit count wraps to
// zero (CCSDS 732.0-B-5 §4.1.2.5.5.2). NOTE: aggregate init `{1u}` would set
// ONLY element [0] to 1 and zero the rest, so every VC's cycle is seeded to 1
// explicitly in setup() (see there) to match the previous hard-coded on-air
// value of 1 for all VCs, including idle VC 63.
static uint8_t frame_counter_cycle[VCDU_NUM_VC] = {0u};

// Frequency calibration variables
static double current_freq_hz = 436500000.0;
static double freq_offset_hz = 0.0;
static bool cw_mode = false;

// Calibration clamp limits: keep values inside the int16 RRC table range so
// large calibration values cannot overflow and silently corrupt modulation.
static const int32_t CAL_GAIN_LIMIT   = 1024;   // q_gain_cal / phase_cal
static const int32_t CAL_OFFSET_LIMIT = 8192;   // i_dc_offset / q_dc_offset

// Digital DAC scaling: single default (clamp-branch value) plus hardening range.
static const float DEFAULT_DIGITAL_SCALE = 12000.0f;
static const float DIGITAL_SCALE_MIN = 4000.0f;
static const float DIGITAL_SCALE_MAX = 30000.0f;

static int16_t q_gain_cal = 0;
static int16_t phase_cal = 0;
static int16_t i_dc_offset = 0;
static int16_t q_dc_offset = 0;
static uint8_t dac_gain_idx =
    3; // Default 0 dBFS (max gain, same as original config)
static uint8_t mixer_gain = 14; // Default 14 (0x0E, same as original config)
static float digital_scale = DEFAULT_DIGITAL_SCALE;

static void update_tx_frequency(void) {
  double target_freq = current_freq_hz + freq_offset_hz;
  sx.setFrequency(target_freq);
  Serial.printf("TX Frequency set to: %.4f MHz (offset: %.2f Hz)\n",
                target_freq / 1e6, freq_offset_hz);
}

static void save_calibration_to_eeprom(void) {
  uint32_t magic = 0xDEADC0E1;
  EEPROM.put(0, magic);
  EEPROM.put(4, freq_offset_hz);
  EEPROM.put(12, q_gain_cal);
  EEPROM.put(14, phase_cal);
  EEPROM.put(16, mixer_gain);
  EEPROM.put(17, dac_gain_idx);
  EEPROM.put(18, digital_scale);
  EEPROM.put(22, i_dc_offset);
  EEPROM.put(24, q_dc_offset);
  EEPROM.put(26, symbol_rate_hz);
  EEPROM.put(30, filter_bw_val);
  if (EEPROM.commit()) {
    Serial.println("Saved calibrations to EEPROM:");
    Serial.printf("  Freq offset: %.2f Hz\n", freq_offset_hz);
    Serial.printf("  Q Gain Cal : %d/1024\n", q_gain_cal);
    Serial.printf("  Phase Cal  : %d/1024\n", phase_cal);
    Serial.printf("  I DC Offset: %d\n", i_dc_offset);
    Serial.printf("  Q DC Offset: %d\n", q_dc_offset);
    Serial.printf("  Mixer Gain : %d (%d dB relative)\n", mixer_gain,
                  (int)mixer_gain * 2 - 28);
    Serial.printf("  DAC Gain   : -%d dB (%s)\n", (3 - dac_gain_idx) * 3,
                  dac_gain_idx == 3 ? "0 dBFS rail-to-rail" : "attenuated");
    Serial.printf("  Digit Scale: %.1f\n", digital_scale);
  } else {
    Serial.println("Failed to write EEPROM!");
  }
}

static void print_help(void) {
  Serial.println("Commands:");
  Serial.println("  r <rate> - Set symbol rate (1000-5000000 Hz)");
  Serial.println("  s - Start modulation");
  Serial.println("  p - Stop modulation");
  Serial.println("  i - Print current rate");
  Serial.println("  c - Toggle CCSDS convolutional encoding");
  Serial.println("  k <rate> - Set convolutional puncturing rate (0=1/2, "
                 "1=2/3, 2=3/4, 3=5/6, 4=7/8)");
  Serial.println("  n - Toggle CCSDS randomizer ON/OFF");
  Serial.println("  N - Toggle CCSDS randomizer polynomial (8-bit / 17-bit)");
  Serial.println("  x - Toggle NRZ-M Encoding");
  Serial.println("  q - Print upload/FIFO status");
  Serial.println("  y - Toggle Reed-Solomon (255,223) I=4 encoding");
  Serial.println("  L - Toggle LDPC encoding");
  Serial.println("  e - Toggle Frame Error Control Field (FECF)");
  Serial.println("  u - Binary upload mode (raw Space Packets, timeout 15s exits)");
  Serial.println("  m - Restart whole microcontroller");
  Serial.println("  W - Toggle RRC/RC pulse shaping filter");
  Serial.println("  h - Print this help menu");
  Serial.println("--- SX1255 Specific ---");
  Serial.println("  o - Toggle Modulation Mode (QPSK / OQPSK)");
  Serial.println("  O - Toggle OQPSK Q Delay (Leads I / Lags I)");
  Serial.println("  P - Toggle Polynomial Swap (G1 / G2)");
  Serial.println("  I - Toggle G2 Inversion");
  Serial.println("  S - Toggle IQ Swap");
  Serial.println("  A - Cycle RRC Alpha (0.5 -> 0.35 -> 0.25)");
  Serial.println("  F <min_dac> <span> <alpha> - Set RRC filter parameters");
  Serial.println("  + / -  - Increase/Decrease freq offset by 100 Hz");
  Serial.println("  ] / [  - Increase/Decrease freq offset by 10 Hz");
  Serial.println("  } / {  - Increase/Decrease freq offset by 1.0 Hz");
  Serial.println("  > / <  - Increase/Decrease freq offset by 0.1 Hz");
  Serial.println("  g - Cycle SX1255 DAC Gain (0dB, -3dB, -6dB, -9dB)");
  Serial.println("  f - Cycle SX1255 Mixer Gain (0 to -16dB)");
  Serial.println("  G <dac 0-3> <mixer 0-15> [scale 4000-16000] - Set absolute TX gain");
  Serial.println("  K / ;  - Increase/Decrease Q Gain Calibration");
  Serial.println("  . / ,  - Increase/Decrease IQ Phase Calibration");
  Serial.println("  J / j  - Increase/Decrease I DC Offset by 10");
  Serial.println("  C / d  - Increase/Decrease Q DC Offset by 10");
  Serial.println("  z - Toggle CW (Carrier Wave) Calibration Mode");
  Serial.println("  X - Swap physical I/Q pin routing");
  Serial.println("  v / V  - Decrease/Increase Digital DAC Scaling");
  Serial.println("  b / B  - Decrease/Increase DSB Filter BW (0-31)");
  Serial.println("  w - Save current calibrations to EEPROM");
}

// Diagnostic configuration variables (defined high up for scope visibility)
static bool swap_iq = false;
static bool invert_g2 = false; // Default: false
static bool swap_pins = false; // Default: false (toggled via 'X' command)
static bool use_conv = true;
static bool use_randomizer = true;
static bool use_rs = true;
static bool use_ldpc = false;
static bool use_fecf = true;
static uint32_t randomizer_poly = 8;
static bool use_oqpsk = true;
static bool use_rrc = true;
static bool oqpsk_q_lead = false; // Default: false (Q lags I)
static bool swap_polys = false;
static bool use_nrzm = false;
static float rrc_alpha = 0.5f;
static uint8_t nrzm_state = 0;

static uint8_t puncturing_rate = 0; // 0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8
static uint8_t punc_phase = 0;

// Inter-core status/sync variables
static volatile uint32_t buffers_refilled = 0;
static volatile uint32_t last_refill_duration_us = 0;
static volatile bool dma_ready = false;
static volatile bool reset_request = false;
static volatile bool needs_pattern_init =
    true; // Core 1 waits for Core 0 to fill FIFO before priming patterns

static inline uint8_t encode_nrzm_byte(uint8_t b) {
  uint8_t out = 0;
  for (int bit = 7; bit >= 0; bit--) {
    uint8_t in_bit = (b >> bit) & 1;
    nrzm_state ^= in_bit;
    out = (out << 1) | nrzm_state;
  }
  return out;
}

static inline uint16_t frame_payload_size(void) {
  uint16_t parity_size = (use_rs || use_ldpc) ? FRAME_PARITY_BYTES : 0;
  uint16_t sz =
      (uint16_t)(FRAME_CODED_BYTES - parity_size - VCDU_HEADER_SIZE -
                 MPDU_HEADER_SIZE);
  if (use_fecf) sz -= 2u;  // FECF occupies the last 2 payload bytes
  return sz;
}

static void build_vcdu_header(uint8_t *frame, uint8_t vcid) {
  frame[0] =
      (VCDU_TRANSFER_FRAME_VERSION << 6) | ((VCDU_SPACECRAFT_ID >> 2) & 0x3F);
  frame[1] = ((VCDU_SPACECRAFT_ID & 0x03) << 6) | (vcid & 0x3F);
  uint8_t vc = vcid & 0x3F;
  frame[2] = (frame_counter[vc] >> 16) & 0xFF;
  frame[3] = (frame_counter[vc] >> 8) & 0xFF;
  frame[4] = frame_counter[vc] & 0xFF;
  frame[5] = (VCDU_REPLAY_FLAG << 7) | (VCDU_CYCLE_USE_FLAG << 6) |
             (((VCDU_SPACECRAFT_ID >> 8) & 0x03) << 4) |
             (frame_counter_cycle[vc & 0x3F] & 0x0F);
}

// 131072-bit circular buffer for raw output stream
static uint8_t tx_bit_fifo[TX_FIFO_SIZE]
    __attribute__((section(".uninitialized")));
static volatile uint32_t tx_fifo_wptr = 0;   // write pointer
static volatile uint32_t tx_fifo_rptr_i = 0; // read pointer for I
static volatile uint32_t tx_fifo_rptr_q =
    1; // read pointer for Q (starts at odd index)

// Root Raised Cosine (RRC) lookup table
// 512 sample points within a symbol, 32 possible combinations of the 5
// neighboring symbols.
static int16_t rrc_table_i[32][512];
static int16_t rrc_table_q_main[32][512];
static int16_t rrc_table_q_cross[32][512];

static bool __not_in_flash_func(has_data_to_send)() {
  uint32_t fifo_count = get_sp_fifo_count();
  if (current_sp_offset < current_sp_size) {
    if (is_filler)
      return true;
    uint16_t payload_size = frame_payload_size();
    uint32_t bytes_needed = current_sp_size - current_sp_offset;
    if (bytes_needed > payload_size)
      bytes_needed = payload_size;
    if (fifo_count >= bytes_needed)
      return true;
    return false; // Starved mid-packet!
  }
  if (get_sp_fifo_free() == 0)
    return true; // FIFO is full
  if (fifo_count >= 6) {
    while (get_sp_fifo_count() >= 6) {
      if ((peek_sp_byte(0) & 0xF8) != 0x00) { // version==0, type==0, sec-hdr==0
        pop_sp_byte();
        continue;
      }
      uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
      uint32_t total_len = pdl + 7;
      if (total_len < 7 || total_len > MAX_SP_PACKET_LEN) {
        pop_sp_byte();
        continue;
      }
      if (get_sp_fifo_count() >= total_len)
        return true;
      break;
    }
  }
  return false;
}

static void __not_in_flash_func(generate_next_frame)(void) {
  uint32_t tf_bytes = FRAME_CODED_BYTES;
  static uint8_t local_frame[FULL_PACKET_SIZE];

  uint32_t t_bb = time_us_32();
  bool has_data = has_data_to_send();
  uint8_t vcid = has_data ? VCDU_DEFAULT_VCID : 0x3F;

  build_vcdu_header(local_frame, vcid);

  if (has_data)
    tx_user_frames_generated++;

  uint16_t fhp = MPDU_NO_START_PACKET;
  uint16_t payload_size = frame_payload_size();

  for (uint16_t i = 0; i < payload_size; i++) {
    if (current_sp_offset >= current_sp_size) {
      current_sp_offset = 0;
      current_sp_size = 0;
      is_filler = false;

      while (has_data && get_sp_fifo_count() >= 6) {
        if ((peek_sp_byte(0) & 0xF8) != 0x00) { // version==0, type==0, sec-hdr==0
          pop_sp_byte();
          continue;
        }
        uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
        uint32_t total_len = pdl + 7;
        if (total_len < 7 || total_len > MAX_SP_PACKET_LEN) {
          pop_sp_byte();
          continue;
        }
        if (get_sp_fifo_count() >= total_len) {
          current_sp_size = total_len;
          if (fhp == MPDU_NO_START_PACKET)
            fhp = i;
        }
        break;
      }

      if (current_sp_size == 0) {
        if (i >= payload_size) break;
        uint16_t remaining = payload_size - i;
        uint16_t filler_size = (remaining >= 7) ? remaining : 7;
        current_sp_size = filler_size;
        is_filler = true;
        if (fhp == MPDU_NO_START_PACKET && i == 0) {
          // The M_PDU Packet Zone is entirely idle fill: emit the 'all ones minus
          // one' idle-data sentinel (CCSDS 732.0-B-5 §4.1.4.2.2.5 / §4.1.4.2.3.4).
          // Guard i == 0 so a zone that carries a prior packet/continuation
          // (FHP legitimately 'no start' = 0xFFFF) is not mis-signalled.
          fhp = MPDU_IDLE_DATA;
        }

        filler_header[0] = 0x07;
        filler_header[1] = 0xFF;
        filler_header[2] = 0xC0 | ((filler_seq >> 8) & 0x3F);
        filler_header[3] = filler_seq & 0xFF;
        filler_seq = (filler_seq + 1) & 0x3FFF;
        uint16_t pdl = filler_size - 7;
        filler_header[4] = (pdl >> 8) & 0xFF;
        filler_header[5] = pdl & 0xFF;
      }
    }

    if (is_filler) {
      if (current_sp_offset < 6) {
        local_frame[VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + i] =
            filler_header[current_sp_offset];
      } else {
        local_frame[VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + i] = 0xFF;
      }
      current_sp_offset++;
    } else if (get_sp_fifo_count() > 0) {
      local_frame[VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + i] = pop_sp_byte();
      current_sp_offset++;
    } else {
      // Starved mid-packet: do not consume; emit idle fill and hold packet state
      // so the partially-framed packet resumes in a later call. (CCSDS 732.0-B-5
      // §4.1.4.1.5 / §4.1.4.2.3.4: emit idle fill when no valid data is ready.)
      local_frame[VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + i] = 0xFF;
    }
  }

  // MPDU Header (First Header Pointer is 16 bits in CCSDS 732.0-B-5 §4.1.4.2.2.1)
  local_frame[VCDU_HEADER_SIZE] = (fhp >> 8) & 0xFF; // FHP high byte (bits 15..8)
  local_frame[VCDU_HEADER_SIZE + 1] = fhp & 0xFF;    // FHP low byte (bits 7..0)

  if (use_fecf) {
    // FECF sits at the true end of the uncoded frame: FRAME_PAYLOAD_LIMIT-2 when
    // RS/LDPC is active, otherwise the full uncoded frame FRAME_CODED_BYTES-2.
    uint16_t crc_off = (use_rs || use_ldpc)
                           ? (uint16_t)(FRAME_PAYLOAD_LIMIT - 2)
                           : (uint16_t)(FRAME_CODED_BYTES - 2);
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < crc_off; i++) {
      crc ^= (local_frame[i] << 8);
      for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc = (crc << 1);
      }
    }
    local_frame[crc_off] = (crc >> 8) & 0xFF;
    local_frame[crc_off + 1] = crc & 0xFF;
  }

  // Apply FEC encoding if enabled (overwrites the last FRAME_PARITY_BYTES bytes with parity)
  if (use_ldpc) {
    ldpc_78_encode(local_frame, FRAME_PAYLOAD_LIMIT);
  } else if (use_rs) {
    rs_encode_interleaved(&local_frame[0], &local_frame[FRAME_PAYLOAD_LIMIT], 4);
  }

  // Combine ASM and randomized payload into a single 1024-byte packet
  static uint8_t full_packet[FULL_PACKET_SIZE];

  // ASM is 4 bytes, unrandomized
  full_packet[0] = 0x1A;
  full_packet[1] = 0xCF;
  full_packet[2] = 0xFC;
  full_packet[3] = 0x1D;

  // Payload is 1020 bytes, randomized (if enabled)
  for (uint32_t byte_idx = 0; byte_idx < tf_bytes; byte_idx++) {
    uint8_t payload_byte = local_frame[byte_idx];
    if (use_randomizer) {
      uint8_t rand_byte =
          (randomizer_poly == 17)
              ? CCSDS_RANDOMIZER17.seq[byte_idx]
              : CCSDS_RANDOMIZER[byte_idx % sizeof(CCSDS_RANDOMIZER)];
      payload_byte ^= rand_byte;
    }
    full_packet[4 + byte_idx] = payload_byte;
  }

  if (use_conv) {
    uint32_t local_wptr = tx_fifo_wptr;
    // Process the entire 1024-byte packet continuously
    for (int i = 0; i < FULL_PACKET_SIZE; i++) {
      uint8_t raw_byte = full_packet[i];
      if (use_nrzm) {
        raw_byte = encode_nrzm_byte(raw_byte);
      }

      uint16_t out_16 = (CCSDS_CONV_FAST_LUT.byte_out[raw_byte] ^
                         CCSDS_CONV_FAST_LUT.state_out[conv_state]);
      if (invert_g2) {
        out_16 ^= 0x5555u;
      }
      if (swap_polys) {
        out_16 = ((out_16 & 0xAAAAu) >> 1) | ((out_16 & 0x5555u) << 1);
      }
      conv_state = CCSDS_CONV_FAST_LUT.next_state[raw_byte];

      if (puncturing_rate == 0) {
        for (int bit = 15; bit >= 0; bit--) {
          tx_bit_fifo[local_wptr & TX_FIFO_MASK] = (out_16 >> bit) & 1;
          local_wptr++;
        }
      } else {
        uint8_t hi = out_16 >> 8;
        uint32_t p_hi =
            CCSDS_PUNC_FAST_LUT.table[puncturing_rate][punc_phase][hi];
        punc_phase = (p_hi >> 16) & 0xFF;
        uint8_t count_hi = (p_hi >> 8) & 0xFF;
        uint8_t bits_hi = p_hi & 0xFF;
        for (int bit = count_hi - 1; bit >= 0; bit--) {
          tx_bit_fifo[local_wptr & TX_FIFO_MASK] = (bits_hi >> bit) & 1;
          local_wptr++;
        }

        uint8_t lo = out_16 & 0xFF;
        uint32_t p_lo =
            CCSDS_PUNC_FAST_LUT.table[puncturing_rate][punc_phase][lo];
        punc_phase = (p_lo >> 16) & 0xFF;
        uint8_t count_lo = (p_lo >> 8) & 0xFF;
        uint8_t bits_lo = p_lo & 0xFF;
        for (int bit = count_lo - 1; bit >= 0; bit--) {
          tx_bit_fifo[local_wptr & TX_FIFO_MASK] = (bits_lo >> bit) & 1;
          local_wptr++;
        }
      }
    }
    tx_fifo_wptr = local_wptr;
  } else {
    uint32_t local_wptr = tx_fifo_wptr;
    // Uncoded branch
    for (int i = 0; i < FULL_PACKET_SIZE; i++) {
      uint8_t raw_byte = full_packet[i];
      if (use_nrzm) {
        raw_byte = encode_nrzm_byte(raw_byte);
      }
      for (int bit = 7; bit >= 0; bit--) {
        tx_bit_fifo[local_wptr & TX_FIFO_MASK] = (raw_byte >> bit) & 1;
        local_wptr++;
      }
    }
    tx_fifo_wptr = local_wptr;
  }

  // Increment the emitted VC's frame counter only (CCSDS 732.0-B-5 §4.1.2.4.2).
  uint8_t vc = vcid & 0x3F;
  frame_counter[vc] = (frame_counter[vc] + 1u) & 0xFFFFFFu;
  if (frame_counter[vc] == 0u) {
    // Frame Count returned to zero: increment this VC's 4-bit Cycle
    // (CCSDS 732.0-B-5 §4.1.2.5.5.2).
    frame_counter_cycle[vc] = (uint8_t)((frame_counter_cycle[vc] + 1u) & 0x0F);
  }
  perf_bb_us = time_us_32() - t_bb;
}

// OQPSK symbol state
static uint8_t i_pattern = 0;
static uint8_t q_pattern = 0;

static int32_t i_timer =
    use_oqpsk ? (oqpsk_q_lead ? cycles_per_symbol : cycles_per_symbol / 2)
              : cycles_per_symbol;
static int32_t q_timer =
    use_oqpsk ? (oqpsk_q_lead ? cycles_per_symbol / 2 : cycles_per_symbol)
              : cycles_per_symbol;

// ============================================================
// Fill one 32-bit PIO FIFO word (16 I/Q bit-pairs, LSB-first).
// Bit layout: [1:0]={Q[0],I[0]}, [3:2]={Q[1],I[1]}, ...
// ============================================================

// ============================================================
// 2nd-order Sigma-Delta modulator state — file-scope so every
// mode-specialized variant shares continuous state.
// ============================================================
static int32_t sd_i_acc1 = 0;
static int32_t sd_i_acc2 = 0;
static int32_t sd_q_acc1 = 0;
static int32_t sd_q_acc2 = 0;
static int32_t fb_i = 0;
static int32_t fb_q = 0;
// 16-bit LFSR dither state — breaks SD limit cycles by injecting ±1 noise
static uint16_t sd_dither = 0xACE1;
static constexpr int32_t SD_FB = 30000;
static constexpr int32_t SD_CLAMP = SD_FB * 3;

// Common prologue: load SD state and cache calibration values in registers
#define SD_STATE_LOAD \
  int32_t acc1_i = sd_i_acc1; \
  int32_t acc2_i = sd_i_acc2; \
  int32_t acc1_q = sd_q_acc1; \
  int32_t acc2_q = sd_q_acc2; \
  int32_t local_fb_i = fb_i; \
  int32_t local_fb_q = fb_q; \
  const int32_t _gain_cal = q_gain_cal; \
  const int32_t _ph_cal = phase_cal; \
  const int32_t _i_dc = i_dc_offset; \
  const int32_t _q_dc = q_dc_offset; \
  /* LFSR dither: inject ±1 into first integrator to break limit cycles */ \
  sd_dither = (sd_dither >> 1) ^ (-(sd_dither & 1) & 0xB400u); \
  acc1_i += (int32_t)((sd_dither & 1) << 1) - 1; \
  acc1_q += (int32_t)((sd_dither & 2)) - 1;

// Common epilogue: anti-windup clamp + write back
#define SD_STATE_SAVE \
  if (acc1_i > SD_CLAMP) acc1_i = SD_CLAMP; \
  else if (acc1_i < -SD_CLAMP) acc1_i = -SD_CLAMP; \
  if (acc2_i > SD_CLAMP) acc2_i = SD_CLAMP; \
  else if (acc2_i < -SD_CLAMP) acc2_i = -SD_CLAMP; \
  if (acc1_q > SD_CLAMP) acc1_q = SD_CLAMP; \
  else if (acc1_q < -SD_CLAMP) acc1_q = -SD_CLAMP; \
  if (acc2_q > SD_CLAMP) acc2_q = SD_CLAMP; \
  else if (acc2_q < -SD_CLAMP) acc2_q = -SD_CLAMP; \
  sd_i_acc1 = acc1_i; \
  sd_i_acc2 = acc2_i; \
  sd_q_acc1 = acc1_q; \
  sd_q_acc2 = acc2_q; \
  fb_i = local_fb_i; \
  fb_q = local_fb_q;

// SWAP_PINS / SWAP_IQ must be compile-time constants (0/1) so the
// mode-specialized variants fully optimize.
#define SD_LOOP_CORE(SWAP_PINS, SWAP_IQ) \
  for (int pair = 0; pair < 16; pair++) { \
    if (__builtin_expect(i_timer == 0, 0)) { \
      tx_fifo_rptr_i += 2; \
      i_pattern = ((i_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_i + 4) & TX_FIFO_MASK]) & 31; \
      i_timer = cycles_per_symbol; \
      if (!SWAP_IQ) { \
          base_i = rrc_table_i[i_pattern]; \
          base_q_cross = rrc_table_q_cross[i_pattern]; \
      } else { \
          base_q_main = rrc_table_q_main[i_pattern]; \
      } \
    } \
    i_timer--; \
    if (__builtin_expect(q_timer == 0, 0)) { \
      tx_fifo_rptr_q += 2; \
      q_pattern = ((q_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_q + 4) & TX_FIFO_MASK]) & 31; \
      q_timer = cycles_per_symbol; \
      if (!SWAP_IQ) { \
          base_q_main = rrc_table_q_main[q_pattern]; \
      } else { \
          base_i = rrc_table_i[q_pattern]; \
          base_q_cross = rrc_table_q_cross[q_pattern]; \
      } \
    } \
    q_timer--; \
    int i_sample_idx = (cycles_per_symbol - 1) - i_timer; \
    int q_sample_idx = (cycles_per_symbol - 1) - q_timer; \
    int32_t val_i_cal = base_i[i_sample_idx]; \
    int16_t val_q_main = base_q_main[q_sample_idx]; \
    int16_t val_q_cross = base_q_cross[i_sample_idx]; \
    int32_t val_q_cal = (int32_t)val_q_main + val_q_cross; \
    acc1_i += val_i_cal - local_fb_i; \
    acc2_i += acc1_i - local_fb_i; \
    uint32_t sign_i = ((uint32_t)acc2_i) >> 31; \
    local_fb_i = sign_i ? -SD_FB : SD_FB; \
    acc1_q += val_q_cal - local_fb_q; \
    acc2_q += acc1_q - local_fb_q; \
    uint32_t sign_q = ((uint32_t)acc2_q) >> 31; \
    local_fb_q = sign_q ? -SD_FB : SD_FB; \
    uint32_t out_i = sign_i ^ 1; \
    uint32_t out_q = sign_q ^ 1; \
    word >>= 2; \
    if (SWAP_PINS) { \
        word |= (out_q | (out_i << 1)) << 30; \
    } else { \
        word |= (out_i | (out_q << 1)) << 30; \
    } \
  }

// Boot-only branchy implementation (used by setup() priming). The compiler
// decides inlining; kept in flash since it never runs on the hot path.
static inline uint32_t build_iq_word_impl(void) {
  // Wait for Core 0 to prime the patterns (on boot and after reset)
  if (needs_pattern_init) {
    return 0; // Output 0s while Core 0 fills the FIFO
  }

  SD_STATE_LOAD;

  uint32_t word = 0;
  const bool _swap_iq = swap_iq;
  const bool _swap_pins = swap_pins;

  if (cw_mode) {
    int32_t vi = (int32_t)digital_scale + _i_dc;
    int32_t vq = (int32_t)digital_scale +
                 (((int32_t)digital_scale * _gain_cal) >> 10) -
                 (((int32_t)digital_scale * _ph_cal) >> 10) + _q_dc;
#pragma GCC unroll 16
    for (int pair = 0; pair < 16; pair++) {
      acc1_i += vi - local_fb_i;
      acc2_i += acc1_i - local_fb_i;
      local_fb_i = (acc2_i < 0) ? -SD_FB : SD_FB;

      acc1_q += vq - local_fb_q;
      acc2_q += acc1_q - local_fb_q;
      local_fb_q = (acc2_q < 0) ? -SD_FB : SD_FB;

      uint8_t out_i = _swap_pins ? (local_fb_q > 0) : (local_fb_i > 0);
      uint8_t out_q = _swap_pins ? (local_fb_i > 0) : (local_fb_q > 0);
      word >>= 2;
      word |= (uint32_t)(out_i | (out_q << 1)) << 30;
    }
  } else if (!use_rrc) {
#pragma GCC unroll 16
    for (int pair = 0; pair < 16; pair++) {
      if (i_timer == 0) {
        tx_fifo_rptr_i += 2;
        i_pattern =
            ((i_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_i + 4) & TX_FIFO_MASK]) &
            31;
        i_timer = cycles_per_symbol;
      }
      i_timer--;
      if (q_timer == 0) {
        tx_fifo_rptr_q += 2;
        q_pattern =
            ((q_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_q + 4) & TX_FIFO_MASK]) &
            31;
        q_timer = cycles_per_symbol;
      }
      q_timer--;

      int16_t val_i = 0; // Bypass filter means raw symbol value
      int16_t val_q = 0;
      int symbol_i = (i_pattern >> 2) & 1;
      int symbol_q = (q_pattern >> 2) & 1;
      int symbol_actual_i = _swap_iq ? symbol_q : symbol_i;
      int symbol_actual_q = _swap_iq ? symbol_i : symbol_q;
      val_i = symbol_actual_i ? digital_scale : -digital_scale;
      val_q = symbol_actual_q ? digital_scale : -digital_scale;

      int32_t val_i_cal = (int32_t)val_i + _i_dc;
      int32_t val_q_cal = (int32_t)val_q +
                          (((int32_t)val_q * _gain_cal) >> 10) -
                          (((int32_t)val_i * _ph_cal) >> 10) + _q_dc;

      acc1_i += val_i_cal - local_fb_i;
      acc2_i += acc1_i - local_fb_i;
      local_fb_i = (acc2_i < 0) ? -SD_FB : SD_FB;

      acc1_q += val_q_cal - local_fb_q;
      acc2_q += acc1_q - local_fb_q;
      local_fb_q = (acc2_q < 0) ? -SD_FB : SD_FB;

      uint8_t out_i = _swap_pins ? (local_fb_q > 0) : (local_fb_i > 0);
      uint8_t out_q = _swap_pins ? (local_fb_i > 0) : (local_fb_q > 0);
      word >>= 2;
      word |= (uint32_t)(out_i | (out_q << 1)) << 30;
    }
  } else {
    // Use the file-scope SD_LOOP_CORE macro (SWAP_PINS, SWAP_IQ are
    // compile-time constants in variant functions, but here they are
    // runtime booleans — the macro still works, just with branches).
    const int16_t* base_i = rrc_table_i[_swap_iq ? q_pattern : i_pattern];
    const int16_t* base_q_main = rrc_table_q_main[_swap_iq ? i_pattern : q_pattern];
    const int16_t* base_q_cross = rrc_table_q_cross[_swap_iq ? q_pattern : i_pattern];

    if (_swap_pins) {
        if (_swap_iq) {
#pragma GCC unroll 16
            SD_LOOP_CORE(1, 1)
        } else {
#pragma GCC unroll 16
            SD_LOOP_CORE(1, 0)
        }
    } else {
        if (_swap_iq) {
#pragma GCC unroll 16
            SD_LOOP_CORE(0, 1)
        } else {
#pragma GCC unroll 16
            SD_LOOP_CORE(0, 0)
        }
    }
  }

  SD_STATE_SAVE;
  return word;
}

// ============================================================
// Mode selection — active_mode is a plain enum (not volatile),
// written by Core 0 and read by Core 1 (loop1) once per refill.
// ============================================================

enum iq_mode {
  MODE_CW = 0,
  MODE_BYPASS,
  MODE_RRC_NORMAL,
  MODE_RRC_SWAPIQ,
  MODE_RRC_SWAPPINS,
  MODE_RRC_SWAPBOTH,
};
static enum iq_mode active_mode = MODE_RRC_NORMAL;
static volatile bool active_mode_dirty = true;

// Call this whenever cw_mode / use_rrc / swap_iq / swap_pins change
static void update_active_builder(void) {
  if (cw_mode)                     { active_mode = MODE_CW; goto dirty; }
  if (!use_rrc)                    { active_mode = MODE_BYPASS; goto dirty; }
  if (swap_pins && swap_iq)        active_mode = MODE_RRC_SWAPBOTH;
  else if (swap_pins)              active_mode = MODE_RRC_SWAPPINS;
  else if (swap_iq)                active_mode = MODE_RRC_SWAPIQ;
  else                             active_mode = MODE_RRC_NORMAL;
dirty:
  __dmb();
  active_mode_dirty = true;
}

// Cold (boot/init) wrapper — not force-inlined, kept in flash
static uint32_t build_iq_word_cold(void) { return build_iq_word_impl(); }

// Helper function to calculate Root Raised Cosine (RRC) filter coefficients.
static float get_rrc_coef(float t, float T, float alpha) {
  if (t == 0.0f) {
    return 1.0f - alpha + (4.0f * alpha / (float)M_PI);
  }
  float limit = T / (4.0f * alpha);
  if (fabsf(t - limit) < 1e-4f || fabsf(t + limit) < 1e-4f) {
    return (alpha / sqrtf(2.0f)) *
           ((1.0f + 2.0f / (float)M_PI) * sinf((float)M_PI / (4.0f * alpha)) +
            (1.0f - 2.0f / (float)M_PI) * cosf((float)M_PI / (4.0f * alpha)));
  }
  float pi_t_T = (float)M_PI * t / T;
  float num = sinf(pi_t_T * (1.0f - alpha)) +
              4.0f * alpha * (t / T) * cosf(pi_t_T * (1.0f + alpha));
  float den = pi_t_T * (1.0f - (4.0f * alpha * t / T) * (4.0f * alpha * t / T));
  return num / den;
}

static void init_rrc_table(void) {
  float T = (float)
      cycles_per_symbol; // Correctly align filter time scale with symbol rate
  float alpha = rrc_alpha;

  int32_t gain_cal = q_gain_cal;
  int32_t ph_cal = phase_cal;
  int32_t i_dc = i_dc_offset;
  int32_t q_dc = q_dc_offset;

  for (int p = 0; p < cycles_per_symbol; p++) {
    for (int pattern = 0; pattern < 32; pattern++) {
      float sum = 0.0f;
      for (int j = -2; j <= 2; j++) {
        int bit_idx = 2 - j; // s[n-2] is bit 4, s[n+2] is bit 0
        int symbol_bit = (pattern >> bit_idx) & 1;
        float symbol_val = symbol_bit ? 1.0f : -1.0f;

        // p goes from 0 to T-1, corresponding to the start (-T/2) to the end of
        // the symbol.
        float t = (float)p - (T / 2.0f) - (float)j * T;
        sum += symbol_val * get_rrc_coef(t, T, alpha);
      }
      // Scale to fit 16-bit range with safety headroom (optimized for 16384 SD
      // feedback). Divide by 1.35f to account for RRC peak overshoot and match
      // the peak amplitude of the bypass mode!
      int16_t base_val = (int16_t)((sum / 1.35f) * digital_scale);

      // Saturate each final value to the int16 range before storing so a large
      // calibration never wraps and silently corrupts the modulation.
      int32_t v_i = base_val + i_dc;
      if (v_i > 32767) v_i = 32767; else if (v_i < -32768) v_i = -32768;
      rrc_table_i[pattern][p] = (int16_t)v_i;

      int32_t v_q_main = base_val + (((int32_t)base_val * gain_cal) >> 10) + q_dc;
      if (v_q_main > 32767) v_q_main = 32767; else if (v_q_main < -32768) v_q_main = -32768;
      rrc_table_q_main[pattern][p] = (int16_t)v_q_main;

      int32_t v_q_cross = -(((int32_t)base_val * ph_cal) >> 10);
      if (v_q_cross > 32767) v_q_cross = 32767; else if (v_q_cross < -32768) v_q_cross = -32768;
      rrc_table_q_cross[pattern][p] = (int16_t)v_q_cross;
    }
  }
}

// ============================================================
// Setup
// ============================================================

// Select SX1255 analog filter bandwidth, PLL bandwidth, and DAC FIR taps
// based on the current symbol rate and RRC alpha.
void update_sx1255_dynamic_filters() {
  // 1. PLL Bandwidth — must exceed the modulation bandwidth.
  if (symbol_rate_hz > 400000)       current_pll_bw = TXFE3_PLL_BW_300kHz;
  else if (symbol_rate_hz > 250000)  current_pll_bw = TXFE3_PLL_BW_225kHz;
  else if (symbol_rate_hz > 100000)  current_pll_bw = TXFE3_PLL_BW_150kHz;
  else                               current_pll_bw = TXFE3_PLL_BW_75kHz;

  // 2. Analog DSB Filter BW — set to ~1.5× the signal DSB bandwidth to pass
  // the modulated signal cleanly while rejecting out-of-band sigma-delta noise.
  // Formula: BW_DSB = 17.15/(41 - val) MHz → val = 41 - 17.15/BW_DSB_MHz.
  // Signal DSB BW ≈ symbol_rate × (1+rrc_alpha).
  float signal_bw_dsb_mhz = (float)symbol_rate_hz * (1.0f + rrc_alpha) / 1e6f;
  float target_bw_mhz = signal_bw_dsb_mhz * 1.5f;
  int bw_val = 31;
  if (target_bw_mhz < 1.715f) {
    bw_val = (int)(41.0f - 17.15f / target_bw_mhz);
    if (bw_val < 0)  bw_val = 0;
    if (bw_val > 31) bw_val = 31;
  }
  filter_bw_val = (uint8_t)bw_val;
  sx.setTxFilterBandwidth(current_pll_bw, filter_bw_val);

  // 3. DAC FIR-DAC reconstruction — always 24 taps (widest BW).
  sx.setTxDacBandwidth(TXFE4_DAC_BW_24TAPS);
}

void setup() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  delay(100); // Let voltage settle
  set_sys_clock_khz(380000, true);
  rs_init();

  Serial.begin(921600);
  delay(500);

  EEPROM.begin(512);
  uint32_t magic = 0;
  EEPROM.get(0, magic);
  if (magic == 0xDEADC0E1) {
    EEPROM.get(4, freq_offset_hz);
    EEPROM.get(12, q_gain_cal);
    EEPROM.get(14, phase_cal);
    EEPROM.get(16, mixer_gain);
    EEPROM.get(17, dac_gain_idx);
    EEPROM.get(18, digital_scale);
    EEPROM.get(22, i_dc_offset);
    EEPROM.get(24, q_dc_offset);
    EEPROM.get(26, symbol_rate_hz);
    EEPROM.get(30, filter_bw_val);
    if (symbol_rate_hz == 0 || symbol_rate_hz == 0xFFFFFFFF)
      symbol_rate_hz = 100000;
    if (filter_bw_val > 31)
      filter_bw_val = 0;

    apply_symbol_rate(symbol_rate_hz);

    if (mixer_gain > 15)
      mixer_gain = 14;
    if (dac_gain_idx > 3)
      dac_gain_idx = 3;
    // Clamp any out-of-range loaded calibration values to the same limits
    // used by the serial command handlers.
    q_gain_cal  = constrain(q_gain_cal,  -CAL_GAIN_LIMIT,   CAL_GAIN_LIMIT);
    phase_cal   = constrain(phase_cal,   -CAL_GAIN_LIMIT,   CAL_GAIN_LIMIT);
    i_dc_offset = constrain(i_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    q_dc_offset = constrain(q_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    if (digital_scale < DIGITAL_SCALE_MIN || digital_scale > DIGITAL_SCALE_MAX)
      digital_scale = DEFAULT_DIGITAL_SCALE;
    Serial.printf("Loaded calibrations: FreqOffset=%.2f Hz, QGain=%d/1024, "
                  "Phase=%d/1024, "
                  "MixerGain=%d, DACGainIdx=%d, DigitalScale=%.1f, IDC=%d, "
                  "QDC=%d\n",
                  freq_offset_hz, q_gain_cal, phase_cal, mixer_gain,
                  dac_gain_idx, digital_scale, i_dc_offset, q_dc_offset);
  } else {
    freq_offset_hz = 0.0;
    q_gain_cal = 0;
    phase_cal = 0;
    i_dc_offset = 0;
    q_dc_offset = 0;
    mixer_gain = 14;
    dac_gain_idx = 3;
    // Apply the same clamping to the fallback defaults (values are 0/in-range).
    q_gain_cal  = constrain(q_gain_cal,  -CAL_GAIN_LIMIT,   CAL_GAIN_LIMIT);
    phase_cal   = constrain(phase_cal,   -CAL_GAIN_LIMIT,   CAL_GAIN_LIMIT);
    i_dc_offset = constrain(i_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    q_dc_offset = constrain(q_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    digital_scale = DEFAULT_DIGITAL_SCALE;
    Serial.println("No calibration data in EEPROM (defaulting to 0/defaults)");
  }

  apply_symbol_rate(symbol_rate_hz);

  init_rrc_table();

  Serial.println("RRC Table Debug:");
  for (int p = 0; p < 8; p++) {
      Serial.printf("  p=%d: pat[0]=%d, pat[16]=%d, pat[31]=%d\n", p,
                    rrc_table_i[0][p], rrc_table_i[16][p], rrc_table_i[31][p]);
  }
  delay(500);
  Serial.printf("  FIR taps      : %lu  (CLK_IN=%lu Hz)\n", cycles_per_symbol,
                (uint32_t)clkin_hz);
  Serial.printf("  Frame coded   : %u bytes  (%lu encoded bits)\n",
                FRAME_CODED_BYTES, (uint32_t)FRAME_ENC_BITS);
  Serial.printf("  Frame payload : %u bytes payload\n", FRAME_PAYLOAD_BYTES);
  Serial.printf("  Frame rate    : %.1f frames/s\n",
                (float)symbol_rate_hz / (32.0f + FRAME_ENC_BITS));
  Serial.printf("  ASM           : 0x%08lX\n", (uint32_t)CADU_ASM);

  // --- SPI ---
  spi_init(spi, 1000000);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);
  gpio_init(PIN_RESET);
  gpio_set_dir(PIN_RESET, GPIO_OUT);
  gpio_put(PIN_RESET, 0);
  gpio_init(PIN_DIO0);
  gpio_set_dir(PIN_DIO0, GPIO_IN);

  sx.init(spi, PIN_CS, PIN_RESET);
  sx.reset();

  if (sx.checkVersion()) {
    Serial.println("SX1255 VER: OK");
  } else {
    Serial.println("SX1255 VER: unexpected");
  }

  if (sx.enableXOSC()) {
    Serial.println("XOSC ready");
  } else {
    Serial.println("XOSC timeout!");
  }

  // --- TX frequency: 437 MHz (applied with calibration offset) ---
  update_tx_frequency();

  // --- TX front-end ---
  sx.setTxGain(dac_gain_idx, mixer_gain);
  update_sx1255_dynamic_filters();
  sx.setIismModeA();
  sx.setClockSelectTxDac();
  sx.enableTx();

  if (sx.waitForTxPllLock(10000)) {
    Serial.println("TX PLL locked");
  } else {
    Serial.println("TX PLL lock timeout!");
  }

  // --- Load PIO ---
  uint offset = pio_add_program(pio, &oqpsk_program_default);
  oqpsk_program_init(pio, sm, offset, clkin_hz);

  Serial.printf("PIO loaded. CLK_IN=%.3f MHz on GP%d, I=GP%d, Q=GP%d\n",
                clkin_hz / 1e6f, PIN_CLKIN, PIN_I_IN, PIN_Q_IN);
  Serial.printf("Symbol rate = %lu Hz (OSR=%lu)\n", symbol_rate_hz,
                cycles_per_symbol);

  // Seed every VC's Frame Count Cycle to 1 (portable explicit loop; a partial
  // aggregate initialiser would leave VCs 1..63 at 0). This keeps byte-5 bits
  // 44-47 identical to the previous hard-coded on-air value of 1 for all VCs,
  // including idle VC 63. Must run before the first generate_next_frame().
  for (uint8_t c = 0; c < VCDU_NUM_VC; c++) frame_counter_cycle[c] = 1u;

  // --- Pre-fill bit FIFO with 4 frames to prevent initial underflow ---
  generate_next_frame();
  generate_next_frame();
  generate_next_frame();
  generate_next_frame();

  // --- Pre-fill DMA buffers and start DMA ---
  for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
    dma_buf[0][i] = build_iq_word_cold();
  }
  for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
    dma_buf[1][i] = build_iq_word_cold();
  }
  next_buf_to_fill = 0;
  update_active_builder(); // Select correct mode-specialized variant

  dma_chan0 = dma_claim_unused_channel(true);
  dma_chan1 = dma_claim_unused_channel(true);

  dma_channel_config c0 = dma_channel_get_default_config(dma_chan0);
  channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
  channel_config_set_read_increment(&c0, true);
  channel_config_set_write_increment(&c0, false);
  channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));
  channel_config_set_chain_to(&c0, dma_chan1);

  dma_channel_config c1 = dma_channel_get_default_config(dma_chan1);
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
  channel_config_set_read_increment(&c1, true);
  channel_config_set_write_increment(&c1, false);
  channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));
  channel_config_set_chain_to(&c1, dma_chan0);

  dma_channel_configure(dma_chan0, &c0,
                        &pio0_hw->txf[sm], // Destination
                        dma_buf[0],        // Source
                        DMA_BUF_SIZE,      // Transfer count
                        false              // Don't start yet
  );

  dma_channel_configure(dma_chan1, &c1, &pio0_hw->txf[sm], dma_buf[1],
                        DMA_BUF_SIZE, false);

  // Start DMA channel 0. It immediately fills the PIO FIFO (8 words) and waits.
  dma_channel_start(dma_chan0);

  // NOW enable the State Machine — it starts with a full FIFO and continuous
  // DMA backing.
  pio_sm_set_enabled(pio, sm, true);
  Serial.println("DMA started. Transmitting OQPSK...");
  dma_ready = true;
  print_help();
}

// ============================================================
// Loop: keep FIFO topped up
// ============================================================

// Shared symbols-per-frame value used by the status query and telemetry paths
static uint32_t symbols_per_frame(void) {
  uint32_t n = FRAME_CODED_BYTES * 8;
  if (use_conv) {
    if (puncturing_rate == 0)
      n *= 2;
    else if (puncturing_rate == 1)
      n = (n * 4) / 3;
    else if (puncturing_rate == 2)
      n = (n * 5) / 4;
    else if (puncturing_rate == 3)
      n = (n * 6) / 5;
    else if (puncturing_rate == 4)
      n = (n * 8) / 7;
  }
  return n;
}

static void process_input(char mode, const char *data) {
  if (mode == 'r') {
    uint32_t sr = strtoul(data, NULL, 10);
    if (sr >= 1000 && sr <= 5000000) {
      apply_symbol_rate(sr);
      pio_sm_set_clkdiv(pio, sm,
                        (float)clock_get_hz(clk_sys) / (float)(clkin_hz * 2u));
      init_rrc_table();
      update_sx1255_dynamic_filters();
      reset_request = true;
      Serial.printf("Symbol rate set to %lu Hz\n", symbol_rate_hz);
    } else {
      Serial.println("Invalid symbol rate (must be 1000 - 5000000)");
    }
  } else if (mode == 's') {
    Serial.println("Modulation started (always on)");
  } else if (mode == 'p') {
    Serial.println("Modulation stopped (not implemented)");
  } else if (mode == 'i') {
    Serial.printf("Current symbol rate: %lu Hz\n", symbol_rate_hz);
  } else if (mode == 'c') {
    use_conv = !use_conv;
    Serial.printf("Convolutional coding: %s\n", use_conv ? "ON" : "OFF");
  } else if (mode == 'k') {
    uint32_t val = strtoul(data, NULL, 10);
    if (val <= 4) {
      puncturing_rate = val;
      Serial.printf("Puncturing rate set to: %d\n", puncturing_rate);
    } else {
      Serial.println("Invalid puncturing rate (0-4)");
    }
  } else if (mode == 'n') {
    use_randomizer = !use_randomizer;
    Serial.printf("CCSDS Randomizer: %s\n", use_randomizer ? "ON" : "OFF");
  } else if (mode == 'N') {
    randomizer_poly = (randomizer_poly == 8) ? 17 : 8;
    Serial.printf("Randomizer poly toggled to %u-bit\n", randomizer_poly);
  } else if (mode == 'x') {
    use_nrzm = !use_nrzm;
    Serial.printf("NRZ-M encoding: %s\n", use_nrzm ? "ON" : "OFF");
  } else if (mode == 'y') {
    use_rs = !use_rs;
    if (use_rs) use_ldpc = false;
    Serial.printf("Reed-Solomon (255,223) I=4 encoding: %s\n",
                  use_rs ? "ON" : "OFF");
  } else if (mode == 'L') {
    use_ldpc = !use_ldpc;
    if (use_ldpc) {
      use_rs = false;
      use_conv = false; // Add this to prevent double-encoding!
    }
    Serial.printf("LDPC (8160,7136) encoding: %s\n",
                  use_ldpc ? "ON" : "OFF");
  } else if (mode == 'e') {
    use_fecf = !use_fecf;
    Serial.printf("FECF (CRC-16): %s\n", use_fecf ? "ON" : "OFF");
  } else if (mode == 'u') {
    binary_upload_mode = true;
    clear_sp_fifo();
    last_binary_rx_ms = millis();
    Serial.println(
        "Binary upload mode enabled. Waiting for raw Space Packets...");
    Serial.print("RDY\n");
    Serial.println("Will exit after 15s of inactivity.");
  } else if (mode == 'm') {
    rp2040.reboot();
  } else if (mode == 'o') {
    use_oqpsk = !use_oqpsk;
    Serial.printf("Modulation: %s\n", use_oqpsk ? "OQPSK" : "QPSK");
  } else if (mode == 'O') {
    oqpsk_q_lead = !oqpsk_q_lead;
    Serial.printf("OQPSK Q Delay: %s\n", oqpsk_q_lead ? "Leads I" : "Lags I");
  } else if (mode == 'P') {
    swap_polys = !swap_polys;
    Serial.printf("Polynomial Swap: %s\n", swap_polys ? "ON" : "OFF");
  } else if (mode == 'I') {
    invert_g2 = !invert_g2;
    Serial.printf("G2 Inversion: %s\n", invert_g2 ? "ON" : "OFF");
  } else if (mode == 'S') {
    swap_iq = !swap_iq;
    update_active_builder();
    Serial.printf("I/Q Data Swap: %s\n", swap_iq ? "ON" : "OFF");
  } else if (mode == 'A') {
    if (rrc_alpha > 0.4f) {
      rrc_alpha = 0.35f;
    } else if (rrc_alpha > 0.3f) {
      rrc_alpha = 0.25f;
    } else {
      rrc_alpha = 0.5f;
    }
    init_rrc_table();
    update_sx1255_dynamic_filters();
    Serial.printf("RRC Alpha set to: %.2f\n", (double)rrc_alpha);
  } else if (mode == 'F') {
    int min_dac = 0, span = 8, type = 1;
    float alpha = 0.5f;
    if (sscanf(data, "%d %d %f %d", &min_dac, &span, &alpha, &type) >= 3) {
      if (alpha == 0.25f || alpha == 0.35f || alpha == 0.5f) {
        rrc_alpha = alpha;
      } else {
        rrc_alpha = 0.5f; // fallback
      }
      init_rrc_table();
      update_sx1255_dynamic_filters();
      Serial.printf("RRC Alpha set to: %.2f\n", (double)rrc_alpha);
    }
  } else if (mode == '+') {
    freq_offset_hz += 100.0f;
    update_tx_frequency();
  } else if (mode == '-') {
    freq_offset_hz -= 100.0f;
    update_tx_frequency();
  } else if (mode == ']') {
    freq_offset_hz += 10.0f;
    update_tx_frequency();
  } else if (mode == '[') {
    freq_offset_hz -= 10.0f;
    update_tx_frequency();
  } else if (mode == '}') {
    freq_offset_hz += 1.0f;
    update_tx_frequency();
  } else if (mode == '{') {
    freq_offset_hz -= 1.0f;
    update_tx_frequency();
  } else if (mode == '>') {
    freq_offset_hz += 0.1f;
    update_tx_frequency();
  } else if (mode == '<') {
    freq_offset_hz -= 0.1f;
    update_tx_frequency();
  } else if (mode == 'g') {
    if (dac_gain_idx == 0) {
      dac_gain_idx = 3;
    } else {
      dac_gain_idx--;
    }
    sx.setTxGain(dac_gain_idx, mixer_gain);
    Serial.printf("TX DAC Gain: -%d dB\n", (3 - dac_gain_idx) * 3);
  } else if (mode == 'f') {
    if (mixer_gain <= 6) {
      mixer_gain = 14;
    } else {
      mixer_gain -= 2;
    }
    sx.setTxGain(dac_gain_idx, mixer_gain);
    Serial.printf("TX Mixer Gain: -%d dB\n", (int)mixer_gain * 2 - 28);
  } else if (mode == 'G') {
    // Absolute TX gain setter: G <dac 0-3> <mixer 0-15> [scale 4000-16000]
    int _dac = 3, _mix = 14, _scale = 0;
    int n = sscanf(data, "%d %d %d", &_dac, &_mix, &_scale);
    if (n >= 2) {
      dac_gain_idx = (uint8_t)constrain(_dac, 0, 3);
      mixer_gain = (uint8_t)constrain(_mix, 0, 15);
      sx.setTxGain(dac_gain_idx, mixer_gain);
      if (n >= 3) {
        float ns = constrain((float)_scale, 4000.0f, 16000.0f);
        if (ns != digital_scale) {
          digital_scale = ns;
          init_rrc_table();
        }
      }
      Serial.printf("TX Gain Set: dac=%u mixer=%u scale=%d\n",
                    dac_gain_idx, mixer_gain, (int)digital_scale);
    } else {
      Serial.println(
          "Invalid G command (use: G <dac 0-3> <mixer 0-15> [scale 4000-16000])");
    }
  } else if (mode == 'K') {
    q_gain_cal += 8;
    q_gain_cal = constrain(q_gain_cal, -CAL_GAIN_LIMIT, CAL_GAIN_LIMIT);
    init_rrc_table();
    Serial.printf("Q Gain: %d\n", q_gain_cal);
  } else if (mode == ';') {
    q_gain_cal -= 8;
    q_gain_cal = constrain(q_gain_cal, -CAL_GAIN_LIMIT, CAL_GAIN_LIMIT);
    init_rrc_table();
    Serial.printf("Q Gain: %d\n", q_gain_cal);
  } else if (mode == '.') {
    phase_cal += 8;
    phase_cal = constrain(phase_cal, -CAL_GAIN_LIMIT, CAL_GAIN_LIMIT);
    init_rrc_table();
    Serial.printf("Phase: %d\n", phase_cal);
  } else if (mode == ',') {
    phase_cal -= 8;
    phase_cal = constrain(phase_cal, -CAL_GAIN_LIMIT, CAL_GAIN_LIMIT);
    init_rrc_table();
    Serial.printf("Phase: %d\n", phase_cal);
  } else if (mode == 'J') {
    i_dc_offset += 10;
    i_dc_offset = constrain(i_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    init_rrc_table();
    Serial.printf("I DC: %d\n", i_dc_offset);
  } else if (mode == 'j') {
    i_dc_offset -= 10;
    i_dc_offset = constrain(i_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    init_rrc_table();
    Serial.printf("I DC: %d\n", i_dc_offset);
  } else if (mode == 'C') {
    q_dc_offset += 10;
    q_dc_offset = constrain(q_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    init_rrc_table();
    Serial.printf("Q DC: %d\n", q_dc_offset);
  } else if (mode == 'd') {
    q_dc_offset -= 10;
    q_dc_offset = constrain(q_dc_offset, -CAL_OFFSET_LIMIT, CAL_OFFSET_LIMIT);
    init_rrc_table();
    Serial.printf("Q DC: %d\n", q_dc_offset);
  } else if (mode == 'z') {
    cw_mode = !cw_mode;
    update_active_builder();
    Serial.printf("CW Mode: %s\n", cw_mode ? "ON" : "OFF");
  } else if (mode == 'W') {
    use_rrc = !use_rrc;
    update_active_builder();
    Serial.printf("RRC Filter: %s\n", use_rrc ? "ON" : "BYPASSED");
  } else if (mode == 'X') {
    swap_pins = !swap_pins;
    update_active_builder();
    Serial.printf("Pin Swap: %s\n", swap_pins ? "ON" : "OFF");
  } else if (mode == 'v') {
    if (digital_scale > 4000.0f) {
      digital_scale -= 1000.0f;
      init_rrc_table();
      Serial.printf("Digital Scaling: %d\n", (int)digital_scale);
    }
  } else if (mode == 'V') {
    if (digital_scale < 16000.0f) {
      digital_scale += 1000.0f;
      init_rrc_table();
      Serial.printf("Digital Scaling: %d\n", (int)digital_scale);
    }
  } else if (mode == 'b') {
    if (filter_bw_val > 0)
      filter_bw_val--;
    sx.setTxFilterBandwidth(current_pll_bw, filter_bw_val);
    Serial.printf("DSB Filter BW: %d\n", filter_bw_val);
  } else if (mode == 'B') {
    if (filter_bw_val < 31)
      filter_bw_val++;
    sx.setTxFilterBandwidth(current_pll_bw, filter_bw_val);
    Serial.printf("DSB Filter BW: %d\n", filter_bw_val);
  } else if (mode == 'w') {
    save_calibration_to_eeprom();
    Serial.println("Calibrations saved.");
  } else if (mode == 'q') {
    Serial.println("=== Modulator Status ===");
    Serial.printf("Temp: %.1f C\n", analogReadTemp());
    Serial.printf("Symbol Rate: %lu Hz\n", symbol_rate_hz);
    Serial.printf("FIFO Size: %u bytes\n", SP_FIFO_SIZE);
    Serial.printf("FIFO Buffered: %u bytes\n", get_sp_fifo_count());
    Serial.printf("FIFO Free: %u bytes\n", get_sp_fifo_free());
    Serial.printf("DMA Underflows: %u\n", tx_dma_underflows);
    Serial.printf("VCDUs Transmitted: %u user (VC%u) + %u idle (VC%u)\n",
                  frame_counter[VCDU_DEFAULT_VCID], VCDU_DEFAULT_VCID,
                  frame_counter[0x3F], 0x3F);
    Serial.printf("User Frames Encapsulated: %u\n", tx_user_frames_generated);
    Serial.printf("CPU BB Time: %u us\n", perf_bb_us);
    Serial.printf("CPU Mod Time: %u us\n", last_refill_duration_us);
    Serial.printf("RRC Filter: %s (Type: RRC, alpha=%.2f, span=5 symbols)\n",
                  use_rrc ? "ON" : "BYPASSED", rrc_alpha);
    Serial.printf("Randomizer: %s (Poly: %u-bit)\n",
                  use_randomizer ? "ON" : "OFF", randomizer_poly);
    Serial.printf("Convolutional: %s\n", use_conv ? "ON" : "OFF");
    Serial.printf("Puncturing Rate: %d\n", puncturing_rate);
    Serial.printf("Frame Size: 1024 bytes (incl. 4-byte ASM; AOS transfer frame = 1020 bytes)\n");
    Serial.printf("RS Interleave: 4\n");
    Serial.printf("RS (255,223): %s\n", use_rs ? "ON" : "OFF");
    Serial.printf("LDPC (8160,7136): %s\n", use_ldpc ? "ON" : "OFF");
    Serial.printf("FECF (CRC-16): %s\n", use_fecf ? "ON" : "OFF");

    uint16_t expected_payload = frame_payload_size();
    Serial.printf("Expected Payload: %u bytes\n", expected_payload);

    uint32_t spf = symbols_per_frame();
    uint32_t frame_tx_time_us =
        (uint32_t)((spf * 1000000ULL) /
                   (symbol_rate_hz * (use_oqpsk ? 2 : 1)));
    float core0_headroom_pct =
        (1.0f - ((float)perf_bb_us / (float)frame_tx_time_us)) * 100.0f;

    uint32_t dma_buf_time_us =
        (uint32_t)((DMA_BUF_SIZE * 16ULL * 1000000ULL) / clkin_hz);
    float core1_headroom_pct =
        (1.0f - ((float)last_refill_duration_us / (float)dma_buf_time_us)) *
        100.0f;

    Serial.printf("Core 0 Headroom: %.1f%%\n", core0_headroom_pct);
    Serial.printf("Core 1 Headroom: %.1f%%\n", core1_headroom_pct);
  } else if (mode == 'h') {
    print_help();
  } else {
    Serial.printf("Unknown command: '%c' (type 'h' for help)\n", mode);
  }
}

void loop() {
  if (binary_upload_mode) {
    uint32_t processed_this_loop = 0;
    while (Serial.available() && processed_this_loop < 1024) {
      if (get_sp_fifo_free() > 0) {
        push_sp_byte(Serial.read());
        last_binary_rx_ms = millis();
        processed_this_loop++;
      } else {
        break; // FIFO full, yield back to main loop to generate frames
      }
    }

    static uint32_t last_telemetry_ms = 0;
    if (millis() - last_telemetry_ms >= 1000) {
      last_telemetry_ms = millis();

      uint32_t spf = symbols_per_frame();
      uint32_t frame_tx_time_us =
          (uint32_t)((spf * 1000000ULL) /
                     (symbol_rate_hz * (use_oqpsk ? 2 : 1)));
      float core0_headroom_pct =
          (1.0f - ((float)perf_bb_us / (float)frame_tx_time_us)) * 100.0f;

      uint32_t dma_buf_time_us =
          (uint32_t)((DMA_BUF_SIZE * 16ULL * 1000000ULL) / clkin_hz);
      float core1_headroom_pct =
          (1.0f - ((float)last_refill_duration_us / (float)dma_buf_time_us)) *
          100.0f;

      Serial.printf("[MCU] FIFO: %u/%u | C0 HDRM: %.1f%% | C1 HDRM: %.1f%% | Temp: %.1fC\n",
                    get_sp_fifo_count(), SP_FIFO_SIZE, core0_headroom_pct,
                    core1_headroom_pct, analogReadTemp());
    }

    if (millis() - last_binary_rx_ms > 15000) {
      binary_upload_mode = false;
      Serial.println("\nExiting binary upload mode due to 15s timeout.");
      clear_sp_fifo();
    }
    // DO NOT return here! We need generate_next_frame() below to run!
  } else {

    while (Serial.available()) {
      char c = Serial.read();

      if (!waiting_for_input) {
        if (c == 'r' || c == 'k' || c == 'l' || c == 't' || c == 'M' || c == 'F' ||
            c == 'G') {
          input_mode = c;
          waiting_for_input = true;
          input_idx = 0;
          input_buffer[0] = '\0';
          Serial.print(c);
          Serial.print(' ');
        } else if (c >= ' ' && c <= '~') {
          process_input(c, "");
        }
      } else {
        if (c == '\n' || c == '\r') {
          input_buffer[input_idx] = '\0';
          Serial.println();
          process_input(input_mode, input_buffer);
          waiting_for_input = false;
        } else if (c == '\b' || c == 127) { // Backspace
          if (input_idx > 0) {
            input_idx--;
            Serial.print("\b \b");
          }
        } else if (input_idx < INPUT_BUFFER_SIZE - 1 && c >= ' ' && c <= '~') {
          input_buffer[input_idx++] = c;
          Serial.print(c);
        }
      } // closes if (!waiting_for_input)
    } // closes while (Serial.available())
  } // closes else (not in binary upload mode)

  // Refill the bit FIFO for core 1
  uint32_t rptr =
      (tx_fifo_rptr_i < tx_fifo_rptr_q) ? tx_fifo_rptr_i : tx_fifo_rptr_q;
  // Diagnostic: track the minimum tx_bit_fifo headroom Core 0 observes.
  int32_t headroom = (int32_t)(tx_fifo_wptr - rptr);
  if (headroom < tx_bit_min_headroom) tx_bit_min_headroom = headroom;
  if ((tx_fifo_wptr - rptr) < 65536) {
    // Top the ring up to the 65536-symbol watermark in one pass (bounded). A
    // frame is <= 16384 symbols, so <= 8 iterations reach the watermark and the
    // ring (131072) can never overflow (65536 + 16383 < 131072).
    for (int i = 0; i < 8 && (tx_fifo_wptr - rptr) < 65536; i++) {
      generate_next_frame();
    }
  }
}

// ============================================================
// Core 1: Dedicated high-speed DMA buffer refilling
void setup1() {
  // Initialization done on Core 0
}

// NOTE: Must be RAM-resident on RP2350. The 6 inlined loop bodies (each
// DMA_BUF_SIZE × 16 unrolled) exceed the instruction cache, so flash
// execution causes I-cache thrashing and catastrophic headroom loss.
void __not_in_flash_func(loop1)() {
  if (!dma_ready) {
    delay(1);
    return;
  }

  if (reset_request) {
    // Reset pointers and request pattern re-initialization
    tx_fifo_wptr = 0;
    tx_fifo_rptr_i = 0;
    tx_fifo_rptr_q = 1;
    i_timer = use_oqpsk
                  ? (oqpsk_q_lead ? cycles_per_symbol : cycles_per_symbol / 2)
                  : cycles_per_symbol;
    q_timer = use_oqpsk
                  ? (oqpsk_q_lead ? cycles_per_symbol / 2 : cycles_per_symbol)
                  : cycles_per_symbol;
    // punc_phase is owned by Core 0 and reset at every frame start in
    // generate_next_frame(); Core 1 must not write it (removed).
    needs_pattern_init =
        true; // Ask Core 0 to fill FIFO before we prime patterns
    reset_request = false;
    return; // Give Core 0 time to fill FIFO before we start consuming
  }

  // If Core 0 hasn't primed patterns yet, spin until it does
  if (needs_pattern_init) {
    __dmb(); // Ensure wptr updated by Core 0 is visible
    uint32_t rptr_min =
        (tx_fifo_rptr_i < tx_fifo_rptr_q) ? tx_fifo_rptr_i : tx_fifo_rptr_q;
    if ((tx_fifo_wptr - tx_fifo_rptr_i) < 49152) {
      return; // Not enough data yet
    }
    // Prime 5-symbol shift-register patterns from live FIFO data
    i_pattern = 0;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i - 4) & TX_FIFO_MASK] << 4;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i - 2) & TX_FIFO_MASK] << 3;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i) & TX_FIFO_MASK] << 2;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i + 2) & TX_FIFO_MASK] << 1;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i + 4) & TX_FIFO_MASK];
    q_pattern = 0;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q - 4) & TX_FIFO_MASK] << 4;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q - 2) & TX_FIFO_MASK] << 3;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q) & TX_FIFO_MASK] << 2;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q + 2) & TX_FIFO_MASK] << 1;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q + 4) & TX_FIFO_MASK];
    needs_pattern_init = false;
  }

  // Diagnostic: has the reader overtaken the writer since the last check?
  // A negative (wrapped) headroom means tx_bit_fifo underflowed and the
  // emitted symbol stream silently slipped at the fixed symbol clock.
  if ((int32_t)(tx_fifo_wptr - tx_fifo_rptr_i) < 0 ||
      (int32_t)(tx_fifo_wptr - tx_fifo_rptr_q) < 0) {
    tx_bit_underflows++;
  }

  int active_chan = (next_buf_to_fill == 0) ? dma_chan0 : dma_chan1;
  if (!dma_channel_is_busy(active_chan)) {
    int other_chan = (next_buf_to_fill == 0) ? dma_chan1 : dma_chan0;
    if (!dma_channel_is_busy(other_chan)) {
      tx_dma_underflows++;
    }
    uint32_t t_start = micros();
    // Memory barrier: active_mode is written by Core 0, read here.
    // Skip the barrier when the mode hasn't changed (the common case).
    if (active_mode_dirty) {
      __dmb();
      active_mode_dirty = false;
    }
    // Refill this buffer — switch on mode once per buffer, not per word
    switch (active_mode) {
      case MODE_CW: {
        SD_STATE_LOAD;
        int32_t vi = (int32_t)digital_scale + _i_dc;
        int32_t vq = (int32_t)digital_scale +
                     (((int32_t)digital_scale * _gain_cal) >> 10) -
                     (((int32_t)digital_scale * _ph_cal) >> 10) + _q_dc;
        const bool _swap_pins = swap_pins;
        for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
          uint32_t word = 0;
#pragma GCC unroll 16
          for (int pair = 0; pair < 16; pair++) {
            acc1_i += vi - local_fb_i;
            acc2_i += acc1_i - local_fb_i;
            local_fb_i = (acc2_i < 0) ? -SD_FB : SD_FB;
            acc1_q += vq - local_fb_q;
            acc2_q += acc1_q - local_fb_q;
            local_fb_q = (acc2_q < 0) ? -SD_FB : SD_FB;
            uint8_t out_i = _swap_pins ? (local_fb_q > 0) : (local_fb_i > 0);
            uint8_t out_q = _swap_pins ? (local_fb_i > 0) : (local_fb_q > 0);
            word >>= 2;
            word |= (uint32_t)(out_i | (out_q << 1)) << 30;
          }
          dma_buf[next_buf_to_fill][i] = word;
        }
        SD_STATE_SAVE;
        break;
      }
      case MODE_BYPASS: {
        SD_STATE_LOAD;
        const bool _swap_iq = swap_iq;
        const bool _swap_pins = swap_pins;
        const int32_t ds = (int32_t)digital_scale;
        for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
          uint32_t word = 0;
#pragma GCC unroll 16
          for (int pair = 0; pair < 16; pair++) {
            if (i_timer == 0) {
              if ((int32_t)(tx_fifo_wptr - tx_fifo_rptr_i) > 7) {
                tx_fifo_rptr_i += 2;
                i_pattern = ((i_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_i + 4) & TX_FIFO_MASK]) & 31;
              }
              i_timer = cycles_per_symbol;
            }
            i_timer--;
            if (q_timer == 0) {
              if ((int32_t)(tx_fifo_wptr - tx_fifo_rptr_q) > 6) {
                tx_fifo_rptr_q += 2;
                q_pattern = ((q_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_q + 4) & TX_FIFO_MASK]) & 31;
              }
              q_timer = cycles_per_symbol;
            }
            q_timer--;
            int symbol_i = (i_pattern >> 2) & 1;
            int symbol_q = (q_pattern >> 2) & 1;
            int symbol_actual_i = _swap_iq ? symbol_q : symbol_i;
            int symbol_actual_q = _swap_iq ? symbol_i : symbol_q;
            int32_t val_i = symbol_actual_i ? ds : -ds;
            int32_t val_q = symbol_actual_q ? ds : -ds;
            int32_t val_i_cal = val_i + _i_dc;
            int32_t val_q_cal = val_q + ((val_q * _gain_cal) >> 10) - ((val_i * _ph_cal) >> 10) + _q_dc;
            acc1_i += val_i_cal - local_fb_i;
            acc2_i += acc1_i - local_fb_i;
            local_fb_i = (acc2_i < 0) ? -SD_FB : SD_FB;
            acc1_q += val_q_cal - local_fb_q;
            acc2_q += acc1_q - local_fb_q;
            local_fb_q = (acc2_q < 0) ? -SD_FB : SD_FB;
            uint8_t out_i = _swap_pins ? (local_fb_q > 0) : (local_fb_i > 0);
            uint8_t out_q = _swap_pins ? (local_fb_i > 0) : (local_fb_q > 0);
            word >>= 2;
            word |= (uint32_t)(out_i | (out_q << 1)) << 30;
          }
          dma_buf[next_buf_to_fill][i] = word;
        }
        SD_STATE_SAVE;
        break;
      }
      // Intentionally hand-duplicated for maximum performance
      case MODE_RRC_NORMAL: {
        SD_STATE_LOAD;
        for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
          uint32_t word = 0;
          const int16_t* base_i = rrc_table_i[i_pattern];
          const int16_t* base_q_main = rrc_table_q_main[q_pattern];
          const int16_t* base_q_cross = rrc_table_q_cross[i_pattern];
#pragma GCC unroll 16
          SD_LOOP_CORE(0, 0);
          dma_buf[next_buf_to_fill][i] = word;
        }
        SD_STATE_SAVE;
        break;
      }
      case MODE_RRC_SWAPIQ: {
        SD_STATE_LOAD;
        for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
          uint32_t word = 0;
          const int16_t* base_i = rrc_table_i[q_pattern];
          const int16_t* base_q_main = rrc_table_q_main[i_pattern];
          const int16_t* base_q_cross = rrc_table_q_cross[q_pattern];
#pragma GCC unroll 16
          SD_LOOP_CORE(0, 1);
          dma_buf[next_buf_to_fill][i] = word;
        }
        SD_STATE_SAVE;
        break;
      }
      case MODE_RRC_SWAPPINS: {
        SD_STATE_LOAD;
        for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
          uint32_t word = 0;
          const int16_t* base_i = rrc_table_i[i_pattern];
          const int16_t* base_q_main = rrc_table_q_main[q_pattern];
          const int16_t* base_q_cross = rrc_table_q_cross[i_pattern];
#pragma GCC unroll 16
          SD_LOOP_CORE(1, 0);
          dma_buf[next_buf_to_fill][i] = word;
        }
        SD_STATE_SAVE;
        break;
      }
      case MODE_RRC_SWAPBOTH: {
        SD_STATE_LOAD;
        for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
          uint32_t word = 0;
          const int16_t* base_i = rrc_table_i[q_pattern];
          const int16_t* base_q_main = rrc_table_q_main[i_pattern];
          const int16_t* base_q_cross = rrc_table_q_cross[q_pattern];
#pragma GCC unroll 16
          SD_LOOP_CORE(1, 1);
          dma_buf[next_buf_to_fill][i] = word;
        }
        SD_STATE_SAVE;
        break;
      }
    }
    last_refill_duration_us = micros() - t_start;

    // Re-arm the channel (trigger = false: it waits for the chain trigger)
    dma_channel_set_read_addr(active_chan, dma_buf[next_buf_to_fill], false);
    dma_channel_set_trans_count(active_chan, DMA_BUF_SIZE, false);

    buffers_refilled++;
    next_buf_to_fill = 1 - next_buf_to_fill;
  }
}