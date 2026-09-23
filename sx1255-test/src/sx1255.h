#pragma once

#include <Arduino.h>
#include <hardware/spi.h>

// TXFE3 PLL BW constants
#define TXFE3_PLL_BW_75kHz 0x0
#define TXFE3_PLL_BW_150kHz 0x1
#define TXFE3_PLL_BW_225kHz 0x2
#define TXFE3_PLL_BW_300kHz 0x3

// TXFE3 Filter BW constants
#define TXFE3_FILTER_BW_209kHz 0x00

// TXFE4 DAC BW constants
#define TXFE4_DAC_BW_24TAPS 0x00
#define TXFE4_DAC_BW_32TAPS 0x01
#define TXFE4_DAC_BW_40TAPS 0x02
#define TXFE4_DAC_BW_64TAPS 0x05

class SX1255 {
public:
  SX1255();

  void init(spi_inst_t *spi, uint cs_pin, uint reset_pin);
  bool checkVersion();
  void reset();
  bool enableXOSC();
  void setFrequency(double freq_hz);
  void setTxGain(uint8_t dac_gain_idx, uint8_t mixer_gain);
  void setTxFilterBandwidth(uint8_t pll_bw, uint8_t filter_bw);
  void setTxDacBandwidth(uint8_t taps);
  void setIismModeA();
  void setClockSelectTxDac();
  void enableTx();
  bool waitForTxPllLock(uint32_t timeout_ms = 1000);

private:
  void writeReg(uint8_t addr, uint8_t val);
  uint8_t readReg(uint8_t addr);

  spi_inst_t *_spi;
  uint _cs_pin;
  uint _reset_pin;
};
