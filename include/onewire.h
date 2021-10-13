#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define ONEWIRE_DDR     DDRB
#define ONEWIRE_PORT    PORTB
#define ONEWIRE_PIN     PINB

#define ONEWIRE_CONNECT 2

#define ONEWIRE_LOW() ONEWIRE_DDR |= (1 << ONEWIRE_CONNECT)
#define ONEWIRE_RELEASE() ONEWIRE_DDR &= ~(1 << ONEWIRE_CONNECT)
#define ONEWIRE_IN()  (ONEWIRE_PIN & (1 << ONEWIRE_CONNECT))

#define TIMESLOT  120
#define GAP 1

const uint8_t SEARCH_ROM = 0xF0;
const uint8_t READ_ROM = 0x33;
const uint8_t MATCH_ROM = 0x55;
const uint8_t SKIP_ROM = 0xCC;

struct __attribute__((__packed__)) find_t {
  static const uint8_t LAST_DEVICE = 0xFF;

  uint8_t rom[8];
  uint8_t last_discrepancy;
};

void onewire_init() {
  ONEWIRE_DDR &= ~(1 << ONEWIRE_CONNECT);
  ONEWIRE_PORT &= ~(1 << ONEWIRE_CONNECT);
}

bool onewire_reset() {
  ONEWIRE_LOW();
  _delay_us(480);
  ONEWIRE_RELEASE();
  _delay_us(70);
  if (! ONEWIRE_IN()) {
    _delay_us(480 - 70);
    return true;
  }
  return false;
}

static void onewire_outbit(bool bit) {
  cli();
  ONEWIRE_LOW();
  if (bit) {
    _delay_us(2);
    ONEWIRE_RELEASE();
    _delay_us(TIMESLOT - 2);
  } else {
    _delay_us(TIMESLOT);
    ONEWIRE_RELEASE();
  }
  sei();
  _delay_us(GAP);
}

static bool onewire_inbit() {
  bool result;

  cli();
  ONEWIRE_LOW();
  _delay_us(2);
  ONEWIRE_RELEASE();
  _delay_us(14 - 2);
  result = ONEWIRE_IN() != 0;
  _delay_us(TIMESLOT - 14);
  sei();
  _delay_us(GAP);
  return result;
}

void onewire_write(uint8_t b) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    onewire_outbit(b & 0x01);
    b >>= 1;
  }
}

void onewire_write(const uint8_t *buf, uint8_t size) {
  while (size--) {
    onewire_write(*buf++);
  }
}

void onewire_write_P(const uint8_t *buf, uint8_t size) {
  while (size--) {
    onewire_write(pgm_read_byte(buf++));
  }
}

uint8_t onewire_read() {
  uint8_t result = 0;

  for (uint8_t bit = 0; bit < 8; ++bit) {
    result >>= 1;
    if (onewire_inbit())
      result |= 0x80;
  }
  return result;
}

void onewire_read(uint8_t *buf, uint8_t size) {
  while (size--) {
    *buf++ = onewire_read();
  }
}

uint8_t onewire_crc8(const uint8_t *buf, uint8_t size) {
  uint8_t crc = 0;
  uint8_t b;

  while (size--) {
    b = *buf++;
		for (uint8_t i = 8; i > 0; --i) {
			uint8_t mix = (crc ^ b) & 0x01;

			crc >>= 1;
			if (mix)
        crc ^= 0x8C;
			b >>= 1;
		}
  }
  return crc;
}

static bool onewire_find(find_t *find) {
  uint8_t len = 1;
  uint8_t pos = 0;
  uint8_t bits = 0;
  uint8_t last_zero = 0;

  onewire_write(SEARCH_ROM);
  while (len <= 64) {
    bool bit, nbit;

    bit = onewire_inbit();
    nbit = onewire_inbit();
    if (bit == nbit) { // Discrepancy
      if (bit) { // No answer
        find->last_discrepancy = 0;
        return false;
      }
      if (len < find->last_discrepancy)
        bit = (find->rom[pos] & (1 << bits)) != 0;
      else
        bit = len == find->last_discrepancy;
      if (! bit)
        last_zero = len;
    }
    if (bit)
      find->rom[pos] |= (1 << bits);
    else
      find->rom[pos] &= ~(1 << bits);
    onewire_outbit(bit);
    if (++bits >= 8) {
      bits = 0;
      ++pos;
    }
    ++len;
  }
  if (onewire_crc8(find->rom, 7) != find->rom[7]) { // CRC error!
    find->last_discrepancy = 0;
    return false;
  }
  if (last_zero)
    find->last_discrepancy = last_zero;
  else
    find->last_discrepancy = find_t::LAST_DEVICE;
  return true;
}

const bool onewire_findfirst(find_t *find) {
  if (onewire_reset()) {
    find->last_discrepancy = 0;
    return onewire_find(find);
  }
  return false;
}

const bool onewire_findnext(find_t *find) {
  if (find->last_discrepancy != find_t::LAST_DEVICE) {
    if (onewire_reset()) {
      return onewire_find(find);
    }
  }
  return false;
}

class OneWire {
public:
  static void begin() {
    onewire_init();
  }
  static bool reset() {
    return onewire_reset();
  }
  static void write(uint8_t b) {
    onewire_write(b);
  }
  static void write(const uint8_t *buf, uint8_t size) {
    onewire_write(buf, size);
  }
  static void write_P(const uint8_t *buf, uint8_t size) {
    onewire_write_P(buf, size);
  }
  static uint8_t read() {
    return onewire_read();
  }
  static void read(uint8_t *buf, uint8_t size) {
    onewire_read(buf, size);
  }
  static uint8_t crc8(const uint8_t *buf, uint8_t size) {
    return onewire_crc8(buf, size);
  }
  static bool findfirst(find_t *find) {
    return onewire_findfirst(find);
  }
  static bool findnext(find_t *find) {
    return onewire_findnext(find);
  }
};
