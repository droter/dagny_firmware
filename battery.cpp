
#include <stdint.h>

extern "C" {
#include "drivers/power.h"
}

#include "publish.h"

extern volatile uint32_t ticks;
uint32_t last_battery = 0;

Publisher<8> battery('B');

void battery_spinOnce() {
   // publish battery data once per second
   if( ticks - last_battery > 1000 ) {
      last_battery = ticks;
      uint8_t main = main_battery();
      uint8_t motor = motor_battery();
      if(battery.reset()) {
         battery.append(main);
         battery.append(motor);
         battery.finish();
      }
   }
}
