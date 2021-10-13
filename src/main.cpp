#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "uart.h"
#include "onewire.h"
#include "ds1820.h"

UART uart;

int main() {
  find_t find;
  bool found;
  uint8_t cnt;

/*
  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
*/

  uart.begin();

//  onewire_init();
  cnt = 0;
  found = onewire_findfirst(&find);
  while (found) {
    if (find.rom[0] == 0x28) { // DS18B20
      ds1820_init(find.rom);
      ++cnt;
    }
    found = onewire_findnext(&find);
  }
  if (cnt) {
    for (;;) {
      ds1820_update();
      _delay_ms(1000);
//      _delay_ms(100);
//      cnt = 1;
      found = onewire_findfirst(&find);
      while (found) {
        if (find.rom[0] == 0x28) { // DS18B20
          int8_t t;

          t = ds1820_read(find.rom);
//          uart.print(cnt++);
//          uart.print_P(PSTR(": "));
          if (t != DS1820_ERROR) {
            uart.print(t);
          } else {
            uart.write('!');
          }
          uart.println();
        }
        found = onewire_findnext(&find);
      }
//      _delay_ms(1000 - 100);
    }
//  } else {
//    uart.print_P(PSTR("No DS18B20!\r\n"));
  }
  for (;;) {}

/*
  cli();
  sleep_enable();
  sleep_cpu();
*/
}
