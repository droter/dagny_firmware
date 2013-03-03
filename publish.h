/*
 * publish.h
 *
 * A wrapper around my protocol to make sending messages easier.
 *
 * Author: Austin Hendrix
 */

#ifndef PUBLISH_H
#define PUBLISH_H

#include <protocol.h>
extern "C" {
#include "drivers/serial.h"
#include "drivers/led.h"
};

extern uint8_t pub_enable;

// experimental publisher class; wraps the Packet class
// targeted to my AVR
template<uint8_t SZ, uint8_t SER = BRAIN>
class Publisher {
   private:
      uint16_t sz;
      char buffer[SZ];
      Packet p;

   public:
      Publisher(char topic) : p(topic, (uint8_t)SZ, buffer) {
         sz = 0;
      }

      int8_t reset() {
         if( sz > 0 ) {
            led_on();
            return 0;
         } else {
            p.reset();
            return 1;
         }
      }

      void finish() {
         p.finish();
         if( pub_enable ) {
            sz = p.outsz();
            tx_buffer(SER, (const uint8_t *)buffer, &sz);
         }
      }

      template<class T> void append(T t) {
         p.append(t);
      }
};

#endif
