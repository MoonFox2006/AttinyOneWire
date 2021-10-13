#pragma once

#include "onewire.h"

const uint8_t DS1820_CONVERT = 0x44;
const uint8_t DS1820_READ = 0xBE;
const uint8_t DS1820_WRITE = 0x4E;
const uint8_t DS1820_COPY = 0x48;
const uint8_t DS1820_RECALL = 0xB8;
const uint8_t DS1820_POWER = 0xB4;

const int8_t DS1820_ERROR = -128;

bool ds1820_init(const uint8_t *rom = nullptr) {
//  onewire_init();

  if (onewire_reset()) {
    uint8_t data[8];

    if (rom) {
      onewire_write(MATCH_ROM);
      onewire_write(rom, 8);
    }
    onewire_write(READ_ROM);
    onewire_read(data, sizeof(data));
    if ((data[0] == 0x28) && (onewire_crc8(data, 7) == data[7])) {
/*
      data[0] = 127;
      data[1] = -128;
      data[2] = 0x1F; // 9 bit
      onewire_write(DS1820_WRITE);
      onewire_write(data, 3);
*/
      return true;
    }
  }
  return false;
}

bool ds1820_update(const uint8_t *rom = nullptr, bool wait = false) {
  if (onewire_reset()) {
    if (rom) {
      onewire_write(MATCH_ROM);
      onewire_write(rom, 8);
    } else {
      onewire_write(SKIP_ROM);
    }
    onewire_write(DS1820_CONVERT);
    if (wait) {
      while (! onewire_inbit()) {}
    }
    return true;
  }
  return false;
}

int8_t ds1820_read(const uint8_t *rom = nullptr) {
  if (onewire_reset()) {
    uint8_t data[9];

    if (rom) {
      onewire_write(MATCH_ROM);
      onewire_write(rom, 8);
    } else {
      onewire_write(SKIP_ROM);
    }
    onewire_write(DS1820_READ);
    onewire_read(data, sizeof(data));
    if (onewire_crc8(data, 8) == data[8]) {
      data[0] >>= 4;
      data[0] |= (data[1] << 4);
      return data[0];
    }
  }
  return DS1820_ERROR;
}

class DS1820 {
public:
  static bool begin(const uint8_t *rom = nullptr) {
    return ds1820_init(rom);
  }
  static bool update(const uint8_t *rom = nullptr, bool wait = false) {
    return ds1820_update(rom, wait);
  }
  static int8_t read(const uint8_t *rom = nullptr) {
    return ds1820_read(rom);
  }
};
