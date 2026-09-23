#include "sx1255.h"

// SX1255 Register Addresses
#define REG_MODE 0x00
#define REG_FRFH_TX 0x04
#define REG_FRFM_TX 0x05
#define REG_FRFL_TX 0x06
#define REG_VERSION 0x07
#define REG_TXFE1 0x08
#define REG_TXFE3 0x0A
#define REG_TXFE4 0x0B
#define REG_CK_SEL 0x10
#define REG_STAT 0x11
#define REG_IISM 0x12

// REG_MODE bits
#define MODE_DRIVER_EN (1 << 3)
#define MODE_TX_EN (1 << 2)
#define MODE_REF_EN (1 << 0)

// STAT bits
#define STAT_XOSC_READY (1 << 2)
#define STAT_PLL_LOCK_TX (1 << 0)

// CK_SEL bits
#define CK_SEL_CLK_SEL_TX_DAC (1 << 0)

// IISM bits
#define IISM_RX_DISABLE (1 << 7)
#define IISM_MODE_A (0x0 << 4)

SX1255::SX1255() : _spi(nullptr), _cs_pin(0), _reset_pin(0) {}

void SX1255::init(spi_inst_t *spi, uint cs_pin, uint reset_pin) {
  _spi = spi;
  _cs_pin = cs_pin;
  _reset_pin = reset_pin;

  gpio_init(_cs_pin);
  gpio_set_dir(_cs_pin, GPIO_OUT);
  gpio_put(_cs_pin, 1);

  gpio_init(_reset_pin);
  gpio_set_dir(_reset_pin, GPIO_OUT);
  gpio_put(_reset_pin, 0);
}

void SX1255::writeReg(uint8_t addr, uint8_t val) {
  uint8_t buf[2] = {(uint8_t)(addr | 0x80u), val};
  gpio_put(_cs_pin, 0);
  spi_write_blocking(_spi, buf, 2);
  gpio_put(_cs_pin, 1);
}

uint8_t SX1255::readReg(uint8_t addr) {
  uint8_t cmd = addr & 0x7Fu;
  uint8_t val = 0;
  gpio_put(_cs_pin, 0);
  spi_write_blocking(_spi, &cmd, 1);
  spi_read_blocking(_spi, 0, &val, 1);
  gpio_put(_cs_pin, 1);
  return val;
}

void SX1255::reset() {
  gpio_put(_reset_pin, 1);
  delayMicroseconds(200);
  gpio_put(_reset_pin, 0);
  delay(10);
}

bool SX1255::checkVersion() { return readReg(REG_VERSION) == 0x11; }

bool SX1255::enableXOSC() {
  writeReg(REG_MODE, MODE_REF_EN);
  uint32_t timeout = 1000;
  while (!(readReg(REG_STAT) & STAT_XOSC_READY) && timeout--) {
    delay(1);
  }
  return (readReg(REG_STAT) & STAT_XOSC_READY);
}

void SX1255::setFrequency(double freq_hz) {
  uint32_t freq_reg = (uint32_t)(freq_hz * 1048576.0 / 32000000.0 + 0.5);
  writeReg(REG_FRFH_TX, (freq_reg >> 16) & 0xFF);
  writeReg(REG_FRFM_TX, (freq_reg >> 8) & 0xFF);
  writeReg(REG_FRFL_TX, freq_reg & 0xFF);
}

void SX1255::setTxGain(uint8_t dac_gain_idx, uint8_t mixer_gain) {
  writeReg(REG_TXFE1, (dac_gain_idx << 4) | (mixer_gain & 0x0F));
}

void SX1255::setTxFilterBandwidth(uint8_t pll_bw, uint8_t filter_bw) {
  writeReg(REG_TXFE3, (pll_bw << 5) | (filter_bw & 0x1F));
}

void SX1255::setTxDacBandwidth(uint8_t taps) {
  writeReg(REG_TXFE4, taps & 0x07);
}

void SX1255::setIismModeA() {
  writeReg(REG_IISM, IISM_RX_DISABLE | IISM_MODE_A);
}

void SX1255::setClockSelectTxDac() {
  writeReg(REG_CK_SEL, 0x00); // Select internal 32 MHz XOSC for DAC clock
}

void SX1255::enableTx() {
  writeReg(REG_MODE, MODE_REF_EN | MODE_TX_EN | MODE_DRIVER_EN);
}

bool SX1255::waitForTxPllLock(uint32_t timeout_ms) {
  while (!(readReg(REG_STAT) & STAT_PLL_LOCK_TX) && timeout_ms--) {
    delay(1);
  }
  return (readReg(REG_STAT) & STAT_PLL_LOCK_TX);
}
