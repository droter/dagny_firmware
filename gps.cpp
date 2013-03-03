/* gps.c
 * GPS library
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>

extern "C" {
#include "drivers/serial.h"
#include "main.h"
}

#include "publish.h"
#include "TinyGPS.h"

uint8_t gps_port;
//gps_simple::SimpleGPS gps_msg;
//ros::Publisher gps_pub("gps", &gps_msg);
//char gps_frame[] = "";
Publisher<16> gps_pub('G');
Publisher<16, BT> gps_pub2('G');

/* initialize GPS listener on serial port */
void gps_init(uint8_t port) {
   // initialize serial port and set baud rate
   serial_init(port);
   serial_baud(port, 57600);

   // set GPS to 57600 baud
   /* uint8_t gps_initstr[] = { 
      0xA0, 0xA1, // header
      0x00, 0x04, // length
      0x05, 0x00, 0x04, 0x01, // payload
      0x00, // checksum
      0x0D, 0x0A // tail
   }; */
   // set GPS to 5 Hz
   uint8_t gps_initstr[] = { 
      0xA0, 0xA1, // header
      0x00, 0x03, // length
      0x0E, 0x05, 0x00, // payload
      0x0B, // checksum 1110 ^ 0101 = 1011 (b
      0x0D, 0x0A // tail
   };
   volatile uint16_t gps_len = sizeof(gps_initstr);
   tx_buffer(port, gps_initstr, (uint16_t*)&gps_len);
   while( gps_len );
   // */

   gps_port = port;
}

// output packet for GPS
//Packet<128> gps_packet('G');
TinyGPS gps;
uint8_t gps_input;
int32_t lat;
int32_t lon;

/* GPS listen thread */
void gps_spinOnce(void) {
   if(rx_ready(gps_port)) {
      gps_input = rx_byte(gps_port);

      if(gps.encode(gps_input)) {
         gps.get_position(&lat, &lon);

         if( gps_pub.reset() ) {
            gps_pub.append(lat);
            gps_pub.append(lon);
            // TODO: fill in rest of GPS message
            gps_pub.finish();
         }

         if( gps_pub2.reset() ) {
            gps_pub2.append(lat);
            gps_pub2.append(lon);
            // TODO: fill in rest of GPS message
            gps_pub2.finish();
         }
      } 
   }
}
