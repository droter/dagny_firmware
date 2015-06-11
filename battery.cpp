
#include <stdint.h>

extern "C" {
#include "drivers/power.h"
}

#include "publish.h"

extern volatile uint32_t ticks;
uint32_t last_battery = 0;

bool motor_cutoff = false;

Publisher<8> battery('B');

const uint8_t main_lvc = 64; // 6.4V; 3.2V per cell
const uint8_t motor_lvc = 64; // 6.4V; 3.2V per cell

void battery_spinOnce() {
   // publish battery data once per second
   if( ticks - last_battery > 1000 ) {
      last_battery = ticks;
      uint8_t main = main_battery();
      uint8_t motor = motor_battery();
      motor_cutoff = (motor <= motor_lvc);

      // if main battery is below cutoff; shut down immediately to
      // protect battery and prevent fire
      if( main <= main_lvc ) pwr_off();

      if(battery.reset()) {
         battery.append(main);
         battery.append(motor);
         battery.finish();
      }
   }
}
