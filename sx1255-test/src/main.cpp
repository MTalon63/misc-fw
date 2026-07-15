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

// Each symbol occupies exactly cycles_per_symbol PIO clock cycles (= CLK_IN
// cycles) We dynamically calculate this to maximize oversampling without
// exceeding SX1255 limits (32 MHz clock)
static uint32_t cycles_per_symbol = 32;
static uint32_t clkin_hz = 0; // Calculated dynamically

// DSB Filter setting (0 to 15). 0 = 418 kHz, 15 = 659 kHz
static uint8_t filter_bw_val = 0;

// QPSK: Q is aligned with I (0 delay cycles)
static constexpr uint32_t Q_OFFSET_CYCLES = 0u;

// PIO / SM
static spi_inst_t *spi = spi0;
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
static constexpr uint32_t FRAME_UNCODED_BITS = (uint32_t)FRAME_CODED_BYTES * 8u;
static constexpr uint32_t FRAME_ENC_BITS = FRAME_UNCODED_BITS * 2u; // rate-1/2

// Convolutional encoder state (persistent across frames for continuous stream)
static uint8_t conv_state = 0u;

// Frame counter (24-bit per CCSDS)
static uint32_t frame_counter = 0u;

// Frequency calibration variables
static double current_freq_hz = 437000000.0;
static double freq_offset_hz = 0.0;
static bool cw_mode = false;

static int16_t q_gain_cal = 0;
static int16_t phase_cal = 0;
static int16_t i_dc_offset = 0;
static int16_t q_dc_offset = 0;
static uint8_t dac_gain_idx =
    3; // Default 0 dBFS (max gain, same as original config)
static uint8_t mixer_gain = 14; // Default 14 (0x0E, same as original config)
static float digital_scale = 3000.0f; // Default 11000

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
  Serial.println("  r <rate> - Set symbol rate (1000-4000000 Hz)");
  Serial.println("  s - Start modulation");
  Serial.println("  p - Stop modulation");
  Serial.println("  i - Print current rate");
  Serial.println("  c - Toggle CCSDS convolutional encoding");
  Serial.println("  k <rate> - Set convolutional puncturing rate (0=1/2, "
                 "1=2/3, 2=3/4, 3=5/6, 4=7/8)");
  Serial.println("  n - Toggle CCSDS randomizer (Not mapped here, fixed)");
  Serial.println("  N - Toggle NRZ-M Encoding");
  Serial.println("  q - Print upload/FIFO status");
  Serial.println("  l <count> - Set Reed-Solomon interleaver depth (Not mapped "
                 "here, fixed 4)");
  Serial.println("  t <data> - Transmit message (ASCII text, Not mapped here)");
  Serial.println("  y - Toggle Reed-Solomon (255,223) I=4 encoding");
  Serial.println("  e - Toggle Frame Error Control Field (FECF) (Not mapped here)");
  Serial.println("  D - Toggle LDPC encoding");
  Serial.println("  u - Binary upload mode (raw Space Packets, timeout 15s exits)");
  Serial.println("  m - Restart whole microcontroller");
  Serial.println("  W - Toggle RRC/RC pulse shaping filter");
  Serial.println("  M <mode> - Set modulation mode (0=BPSK, 1=QPSK, 5=OQPSK, "
                 "Not mapped here)");
  Serial.println("  h - Print this help menu");
  Serial.println("--- SX1255 Specific ---");
  Serial.println("  o - Toggle Modulation Mode (QPSK / OQPSK)");
  Serial.println("  P - Toggle Polynomial Swap (G1 / G2)");
  Serial.println("  I - Toggle G2 Inversion");
  Serial.println("  S - Toggle IQ Swap");
  Serial.println("  A - Cycle RRC Alpha (0.5 -> 0.35 -> 0.25)");
  Serial.println("  + / -  - Increase/Decrease freq offset by 100 Hz");
  Serial.println("  ] / [  - Increase/Decrease freq offset by 10 Hz");
  Serial.println("  } / {  - Increase/Decrease freq offset by 1.0 Hz");
  Serial.println("  > / <  - Increase/Decrease freq offset by 0.1 Hz");
  Serial.println("  X <Hz> - Set SX1255 XTAL frequency (e.g. X 36864000)");
  Serial.println("  g - Cycle SX1255 DAC Gain (0dB, -3dB, -6dB, -9dB)");
  Serial.println("  f - Cycle SX1255 Mixer Gain (0 to -16dB)");
  Serial.println("  K / L  - Increase/Decrease Q Gain Calibration");
  Serial.println("  . / ,  - Increase/Decrease IQ Phase Calibration");
  Serial.println("  J / j  - Increase/Decrease I DC Offset by 10");
  Serial.println("  C / d  - Increase/Decrease Q DC Offset by 10");
  Serial.println("  z - Toggle CW (Carrier Wave) Calibration Mode");
  Serial.println("  x - Swap physical I/Q pin routing");
  Serial.println("  v / V  - Decrease/Increase Digital DAC Scaling");
  Serial.println("  b / B  - Decrease/Increase DSB Filter BW");
  Serial.println("  w - Save current calibrations to EEPROM");
}

// Diagnostic configuration variables (defined high up for scope visibility)
static bool swap_iq = false;
static bool invert_g2 = false; // Default: false
static bool swap_pins = false; // Default: false (toggled via 'x' command)
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

static void build_vcdu_header(uint8_t *frame, uint8_t vcid) {
  frame[0] =
      (VCDU_TRANSFER_FRAME_VERSION << 6) | ((VCDU_SPACECRAFT_ID >> 2) & 0x3F);
  frame[1] = ((VCDU_SPACECRAFT_ID & 0x03) << 6) | (vcid & 0x3F);
  frame[2] = (frame_counter >> 16) & 0xFF;
  frame[3] = (frame_counter >> 8) & 0xFF;
  frame[4] = frame_counter & 0xFF;
  frame[5] = (VCDU_REPLAY_FLAG << 7) | (VCDU_CYCLE_USE_FLAG << 6) |
             ((VCDU_SPACECRAFT_ID >> 8) & 0x03) << 4 |
             (VCDU_FRAME_COUNT_CYCLE & 0x0F);
}

// 131072-bit circular buffer for raw output stream
static uint8_t tx_bit_fifo[131072];
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
    uint16_t length_to_crc = FRAME_CODED_BYTES;
    uint16_t rs_parity_size = use_rs ? 128 : 0;
    uint16_t payload_size =
        length_to_crc - rs_parity_size - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;
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
    if ((peek_sp_byte(0) & 0xE0) != 0x00)
      return true;
    uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
    uint32_t total_len = pdl + 7;
    if (total_len >= SP_FIFO_SIZE)
      return true; // Corrupted length
    if (fifo_count >= total_len)
      return true;
  }
  return false;
}

static void __not_in_flash_func(generate_next_frame)(void) {
  uint32_t tf_bytes = FRAME_CODED_BYTES;
  uint8_t local_frame[1024];

  uint32_t t_bb = time_us_32();
  bool has_data = has_data_to_send();
  uint8_t vcid = has_data ? VCDU_DEFAULT_VCID : 0x3F;

  build_vcdu_header(local_frame, vcid);

  if (has_data)
    tx_user_frames_generated++;

  uint16_t fhp = MPDU_NO_START_PACKET;
  uint16_t parity_size = (use_rs || use_ldpc) ? 128 : 0;
  uint16_t payload_size =
      tf_bytes - parity_size - VCDU_HEADER_SIZE - MPDU_HEADER_SIZE;

  for (uint16_t i = 0; i < payload_size; i++) {
    if (current_sp_offset >= current_sp_size) {
      current_sp_offset = 0;
      current_sp_size = 0;
      is_filler = false;

      while (has_data && get_sp_fifo_count() >= 6) {
        if ((peek_sp_byte(0) & 0xE0) != 0x00) {
          pop_sp_byte();
          continue;
        }
        uint16_t pdl = (peek_sp_byte(4) << 8) | peek_sp_byte(5);
        uint32_t total_len = pdl + 7;
        if (total_len >= SP_FIFO_SIZE) {
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
        uint16_t remaining = payload_size - i;
        uint16_t filler_size = (remaining >= 7) ? remaining : 7;
        current_sp_size = filler_size;
        is_filler = true;
        if (fhp == MPDU_NO_START_PACKET)
          fhp = i;

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
    } else {
      local_frame[VCDU_HEADER_SIZE + MPDU_HEADER_SIZE + i] = pop_sp_byte();
      current_sp_offset++;
    }
  }

  // MPDU Header
  local_frame[VCDU_HEADER_SIZE] = (fhp >> 8) & 0x07;
  local_frame[VCDU_HEADER_SIZE + 1] = fhp & 0xFF;

  if (use_fecf) {
    uint16_t length_to_crc = 892 - 2;
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length_to_crc; i++) {
      crc ^= (local_frame[i] << 8);
      for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc = (crc << 1);
      }
    }
    local_frame[length_to_crc] = (crc >> 8) & 0xFF;
    local_frame[length_to_crc + 1] = crc & 0xFF;
  }

  // Apply FEC encoding if enabled (overwrites the last 128 bytes with parity)
  if (use_ldpc) {
    uint8_t temp[1020];
    ldpc_78_encode(local_frame, 892, temp);
    memcpy(local_frame, temp, 1020);
  } else if (use_rs) {
    rs_encode_interleaved(&local_frame[0], &local_frame[892], 4);
  }

  // Combine ASM and randomized payload into a single 1024-byte packet
  uint8_t full_packet[1024];

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
    for (int i = 0; i < 1024; i++) {
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
          tx_bit_fifo[local_wptr & 131071] = (out_16 >> bit) & 1;
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
          tx_bit_fifo[local_wptr & 131071] = (bits_hi >> bit) & 1;
          local_wptr++;
        }

        uint8_t lo = out_16 & 0xFF;
        uint32_t p_lo =
            CCSDS_PUNC_FAST_LUT.table[puncturing_rate][punc_phase][lo];
        punc_phase = (p_lo >> 16) & 0xFF;
        uint8_t count_lo = (p_lo >> 8) & 0xFF;
        uint8_t bits_lo = p_lo & 0xFF;
        for (int bit = count_lo - 1; bit >= 0; bit--) {
          tx_bit_fifo[local_wptr & 131071] = (bits_lo >> bit) & 1;
          local_wptr++;
        }
      }
    }
    tx_fifo_wptr = local_wptr;
  } else {
    uint32_t local_wptr = tx_fifo_wptr;
    // Uncoded branch
    for (int i = 0; i < 1024; i++) {
      uint8_t raw_byte = full_packet[i];
      if (use_nrzm) {
        raw_byte = encode_nrzm_byte(raw_byte);
      }
      for (int bit = 7; bit >= 0; bit--) {
        tx_bit_fifo[local_wptr & 131071] = (raw_byte >> bit) & 1;
        local_wptr++;
      }
    }
    tx_fifo_wptr = local_wptr;
  }

  // Increment frame counter
  frame_counter = (frame_counter + 1) & 0xFFFFFF;
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

__attribute__((optimize("O3"))) static uint32_t
__not_in_flash_func(build_iq_word)(void) {
  // Wait for Core 0 to prime the patterns (on boot and after reset)
  if (needs_pattern_init) {
    return 0; // Output 0s while Core 0 fills the FIFO
  }

  // 2nd-order Sigma-Delta state variables (cached in registers during loop)
  static int32_t sd_i_acc1 = 0;
  static int32_t sd_i_acc2 = 0;
  static int32_t sd_q_acc1 = 0;
  static int32_t sd_q_acc2 = 0;
  static int32_t fb_i = 0;
  static int32_t fb_q = 0;

  int32_t acc1_i = sd_i_acc1;
  int32_t acc2_i = sd_i_acc2;
  int32_t acc1_q = sd_q_acc1;
  int32_t acc2_q = sd_q_acc2;
  int32_t local_fb_i = fb_i;
  int32_t local_fb_q = fb_q;

  int32_t gain_cal = q_gain_cal;
  int32_t ph_cal = phase_cal;
  int32_t i_dc = i_dc_offset;
  int32_t q_dc = q_dc_offset;

  static constexpr int32_t SD_FB = 30000;
  static constexpr int32_t SD_CLAMP = SD_FB * 3;

  uint32_t word = 0;
  const bool _swap_iq = swap_iq;
  const bool _swap_pins = swap_pins;
  const int32_t _gain_cal = gain_cal;
  const int32_t _ph_cal = ph_cal;
  const int32_t _i_dc = i_dc;
  const int32_t _q_dc = q_dc;

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
            ((i_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_i + 4) & 131071]) &
            31;
        i_timer = cycles_per_symbol;
      }
      i_timer--;
      if (q_timer == 0) {
        tx_fifo_rptr_q += 2;
        q_pattern =
            ((q_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_q + 4) & 131071]) &
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
    const int16_t* base_i = rrc_table_i[_swap_iq ? q_pattern : i_pattern];
    const int16_t* base_q_main = rrc_table_q_main[_swap_iq ? i_pattern : q_pattern];
    const int16_t* base_q_cross = rrc_table_q_cross[_swap_iq ? q_pattern : i_pattern];

#define SD_LOOP_CORE(SWAP_PINS, SWAP_IQ) \
    for (int pair = 0; pair < 16; pair++) { \
      if (__builtin_expect(i_timer == 0, 0)) { \
        tx_fifo_rptr_i += 2; \
        i_pattern = ((i_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_i + 4) & 131071]) & 31; \
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
        q_pattern = ((q_pattern << 1) | tx_bit_fifo[(tx_fifo_rptr_q + 4) & 131071]) & 31; \
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

    if (_swap_pins) {
        if (_swap_iq) {
#pragma GCC unroll 16
            SD_LOOP_CORE(true, true)
        } else {
#pragma GCC unroll 16
            SD_LOOP_CORE(true, false)
        }
    } else {
        if (_swap_iq) {
#pragma GCC unroll 16
            SD_LOOP_CORE(false, true)
        } else {
#pragma GCC unroll 16
            SD_LOOP_CORE(false, false)
        }
    }
  }

  // Save back to RAM (with anti-windup clamping applied once here for low CPU
  // overhead)
  if (acc1_i > SD_CLAMP)
    acc1_i = SD_CLAMP;
  else if (acc1_i < -SD_CLAMP)
    acc1_i = -SD_CLAMP;
  if (acc2_i > SD_CLAMP)
    acc2_i = SD_CLAMP;
  else if (acc2_i < -SD_CLAMP)
    acc2_i = -SD_CLAMP;
  if (acc1_q > SD_CLAMP)
    acc1_q = SD_CLAMP;
  else if (acc1_q < -SD_CLAMP)
    acc1_q = -SD_CLAMP;
  if (acc2_q > SD_CLAMP)
    acc2_q = SD_CLAMP;
  else if (acc2_q < -SD_CLAMP)
    acc2_q = -SD_CLAMP;

  sd_i_acc1 = acc1_i;
  sd_i_acc2 = acc2_i;
  sd_q_acc1 = acc1_q;
  sd_q_acc2 = acc2_q;
  fb_i = local_fb_i;
  fb_q = local_fb_q;

  return word;
}

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

      rrc_table_i[pattern][p] = base_val + i_dc;
      rrc_table_q_main[pattern][p] = base_val + (((int32_t)base_val * gain_cal) >> 10) + q_dc;
      rrc_table_q_cross[pattern][p] = -(((int32_t)base_val * ph_cal) >> 10);
    }
  }
}

// ============================================================
// Setup
// ============================================================

void update_sx1255_dynamic_filters() {
  // 1. Dynamic Analog Filter (DSB BW)
  // Force the widest analog filter (filter_bw_val = 15) which is ~659 kHz.
  // This pushes the analog low-pass cutoff far away from our 150 ksps baseband,
  // drastically reducing group delay / phase distortion on the constellation.
  filter_bw_val = 15;
  sx.setTxFilterBandwidth(TXFE3_PLL_BW_75kHz, filter_bw_val);

  // 2. Dynamic DAC Digital Interpolator Filter
  // Force the widest DAC interpolation filter (24 taps = highest cutoff).
  sx.setTxDacBandwidth(TXFE4_DAC_BW_24TAPS);
}

void setup() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  delay(10); // Let voltage settle
  set_sys_clock_khz(400000, false);
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
    if (filter_bw_val > 15)
      filter_bw_val = 0;

    // Target CLKIN in the optimal RP2040 software limit (~8 Msps)
    cycles_per_symbol = 7500000u / symbol_rate_hz;
    if (cycles_per_symbol > 512)
      cycles_per_symbol = 512; // Max RRC table size
    cycles_per_symbol &= ~1u; // Ensure it's even for perfect OQPSK phase offset
    if (cycles_per_symbol < 2)
      cycles_per_symbol = 2;

    clkin_hz = symbol_rate_hz * cycles_per_symbol;

    if (mixer_gain > 15)
      mixer_gain = 14;
    if (dac_gain_idx > 3)
      dac_gain_idx = 3;
    if (digital_scale < 4000.0f || digital_scale > 30000.0f)
      digital_scale = 12000.0f;
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
    digital_scale = 11000.0f;
    Serial.println("No calibration data in EEPROM (defaulting to 0/defaults)");
  }

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

  // --- Pre-fill bit FIFO with 4 frames to prevent initial underflow ---
  generate_next_frame();
  generate_next_frame();
  generate_next_frame();
  generate_next_frame();

  // --- Pre-fill DMA buffers and start DMA ---
  for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
    dma_buf[0][i] = build_iq_word();
  }
  for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
    dma_buf[1][i] = build_iq_word();
  }
  next_buf_to_fill = 0;

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

static void process_input(char mode, const char *data) {
  if (mode == 'r') {
    uint32_t sr = strtoul(data, NULL, 10);
    if (sr >= 1000 && sr <= 4000000) {
      symbol_rate_hz = sr;
      cycles_per_symbol = 7500000u / symbol_rate_hz;
      if (cycles_per_symbol > 512)
        cycles_per_symbol = 512;
      cycles_per_symbol &= ~1u;
      if (cycles_per_symbol < 2)
        cycles_per_symbol = 2;
      clkin_hz = symbol_rate_hz * cycles_per_symbol;
      pio_sm_set_clkdiv(pio, sm,
                        (float)clock_get_hz(clk_sys) / (float)(clkin_hz * 2u));
      init_rrc_table();
      update_sx1255_dynamic_filters();
      reset_request = true;
      Serial.printf("Symbol rate set to %lu Hz\n", symbol_rate_hz);
    } else {
      Serial.println("Invalid symbol rate (must be 1000 - 4000000)");
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
    if (use_ldpc) use_rs = false;
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
  } else if (mode == 'K') {
    q_gain_cal += 8;
    init_rrc_table();
    Serial.printf("Q Gain: %d\n", q_gain_cal);
  } else if (mode == ';') {
    q_gain_cal -= 8;
    init_rrc_table();
    Serial.printf("Q Gain: %d\n", q_gain_cal);
  } else if (mode == '.') {
    phase_cal += 8;
    init_rrc_table();
    Serial.printf("Phase: %d\n", phase_cal);
  } else if (mode == ',') {
    phase_cal -= 8;
    init_rrc_table();
    Serial.printf("Phase: %d\n", phase_cal);
  } else if (mode == 'J') {
    i_dc_offset += 10;
    init_rrc_table();
    Serial.printf("I DC: %d\n", i_dc_offset);
  } else if (mode == 'j') {
    i_dc_offset -= 10;
    init_rrc_table();
    Serial.printf("I DC: %d\n", i_dc_offset);
  } else if (mode == 'C') {
    q_dc_offset += 10;
    init_rrc_table();
    Serial.printf("Q DC: %d\n", q_dc_offset);
  } else if (mode == 'd') {
    q_dc_offset -= 10;
    init_rrc_table();
    Serial.printf("Q DC: %d\n", q_dc_offset);
  } else if (mode == 'z') {
    cw_mode = !cw_mode;
    Serial.printf("CW Mode: %s\n", cw_mode ? "ON" : "OFF");
  } else if (mode == 'W') {
    use_rrc = !use_rrc;
    Serial.printf("RRC Filter: %s\n", use_rrc ? "ON" : "BYPASSED");
  } else if (mode == 'x') {
    swap_pins = !swap_pins;
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
    sx.setTxFilterBandwidth(TXFE3_PLL_BW_75kHz, filter_bw_val);
    Serial.printf("DSB Filter BW: %d\n", filter_bw_val);
  } else if (mode == 'B') {
    if (filter_bw_val < 15)
      filter_bw_val++;
    sx.setTxFilterBandwidth(TXFE3_PLL_BW_75kHz, filter_bw_val);
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
    Serial.printf("VCDUs Transmitted: %u\n", frame_counter);
    Serial.printf("User Frames Encapsulated: %u\n", tx_user_frames_generated);
    Serial.printf("CPU BB Time: %u us\n", perf_bb_us);
    Serial.printf("CPU Mod Time: %u us\n", last_refill_duration_us);
    Serial.printf("RRC Filter: %s (Type: RRC, alpha=%.2f, span=8)\n",
                  use_rrc ? "ON" : "BYPASSED", rrc_alpha);
    Serial.printf("Randomizer: %s (Poly: %u-bit)\n",
                  use_randomizer ? "ON" : "OFF", randomizer_poly);
    Serial.printf("Convolutional: %s\n", use_conv ? "ON" : "OFF");
    Serial.printf("Puncturing Rate: %d\n", puncturing_rate);
    Serial.printf("Frame Size: 1024 bytes\n");
    Serial.printf("RS Interleave: 4\n");
    Serial.printf("RS (255,223): %s\n", use_rs ? "ON" : "OFF");
    Serial.printf("LDPC (8160,7136): %s\n", use_ldpc ? "ON" : "OFF");
    Serial.printf("FECF (CRC-16): %s\n", use_fecf ? "ON" : "OFF");

    uint16_t parity_size = (use_rs || use_ldpc) ? 128 : 0;
    uint16_t expected_payload = FRAME_CODED_BYTES - parity_size -
                                VCDU_HEADER_SIZE - MPDU_HEADER_SIZE - (use_fecf ? 2 : 0);
    Serial.printf("Expected Payload: %u bytes\n", expected_payload);

    uint32_t symbols_per_frame = FRAME_CODED_BYTES * 8;
    if (use_conv) {
      if (puncturing_rate == 0)
        symbols_per_frame *= 2;
      else if (puncturing_rate == 1)
        symbols_per_frame = (symbols_per_frame * 4) / 3;
      else if (puncturing_rate == 2)
        symbols_per_frame = (symbols_per_frame * 5) / 4;
      else if (puncturing_rate == 3)
        symbols_per_frame = (symbols_per_frame * 6) / 5;
      else if (puncturing_rate == 4)
        symbols_per_frame = (symbols_per_frame * 8) / 7;
    }
    uint32_t frame_tx_time_us =
        (uint32_t)((symbols_per_frame * 1000000ULL) /
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

      uint32_t symbols_per_frame = FRAME_CODED_BYTES * 8;
      if (use_conv) {
        if (puncturing_rate == 0)
          symbols_per_frame *= 2;
        else if (puncturing_rate == 1)
          symbols_per_frame = (symbols_per_frame * 4) / 3;
        else if (puncturing_rate == 2)
          symbols_per_frame = (symbols_per_frame * 5) / 4;
        else if (puncturing_rate == 3)
          symbols_per_frame = (symbols_per_frame * 6) / 5;
        else if (puncturing_rate == 4)
          symbols_per_frame = (symbols_per_frame * 8) / 7;
      }
      uint32_t frame_tx_time_us =
          (uint32_t)((symbols_per_frame * 1000000ULL) /
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
        if (c == 'r' || c == 'k' || c == 'l' || c == 't' || c == 'M' || c == 'F') {
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
  if ((tx_fifo_wptr - rptr) < 65536) {
    generate_next_frame();
  }
}

// ============================================================
// Core 1: Dedicated high-speed DMA buffer refilling
void setup1() {
  // Initialization done on Core 0
}

void loop1() {
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
    punc_phase = 0;
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
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i - 4) & 131071] << 4;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i - 2) & 131071] << 3;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i) & 131071] << 2;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i + 2) & 131071] << 1;
    i_pattern |= tx_bit_fifo[(tx_fifo_rptr_i + 4) & 131071];
    q_pattern = 0;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q - 4) & 131071] << 4;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q - 2) & 131071] << 3;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q) & 131071] << 2;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q + 2) & 131071] << 1;
    q_pattern |= tx_bit_fifo[(tx_fifo_rptr_q + 4) & 131071];
    needs_pattern_init = false;
  }

  int active_chan = (next_buf_to_fill == 0) ? dma_chan0 : dma_chan1;
  if (!dma_channel_is_busy(active_chan)) {
    uint32_t t_start = micros();
    // Refill this buffer
    for (uint32_t i = 0; i < DMA_BUF_SIZE; i++) {
      dma_buf[next_buf_to_fill][i] = build_iq_word();
    }
    last_refill_duration_us = micros() - t_start;

    // Re-arm the channel (trigger = false: it waits for the chain trigger)
    dma_channel_set_read_addr(active_chan, dma_buf[next_buf_to_fill], false);
    dma_channel_set_trans_count(active_chan, DMA_BUF_SIZE, false);

    buffers_refilled++;
    next_buf_to_fill = 1 - next_buf_to_fill;
  }
}