/* power.c
 *
 * Robot power management functions, including:
 *  5V main regulator enable/disable
 *  full power off
 *  battery monitoring
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include "power.h"
#include "adc.h"

/* enable 5V regulator */
void pwr_on() {
   // arduino mega pin 37; PC0
   DDRC |= 1;
   PORTC |= 1;
}

/* disable 5V regulator */
/* for now, don't call this. the regulator sinks current rather than turning off*/
void pwr_sleep() {
   // arduino mega pin 37: PC0
   // FIXME: enable this when we figure out what's going on with the regulator
   DDRC |= 1;
   PORTC &= ~1;
}

/* full system shutdown. there is no going back! */
void pwr_off() {
   // arduino mega pin 48: PL1
   DDRL |= (1 << 1);
   PORTL |= (1 << 1);
}

/* initialize ADCs for reading battries */
void battery_init() {
   // arduino mega pin 48: PL1
   DDRL |= (1 << 1);
   PORTL &= ~(1 << 1); // unset power-down pin

   adc_init();
}

/* read voltage of electronics battery. Volts*10 */
/*
 * Measurement notes:
 *  10.08V: 146
 *   9.51V: 137
 *   9.01V: 129
 *   8.45V: 120
 *   8.02V: 113
 *   7.47V: 104
 *   7.09V: 97
 *   6.79V: 94
 *   6.36V: 95
 *   6.05V: - Arduino browns out :(
 *   5.97V: 97
 *
 * System stable at 6.00V, 1A (idle)
 * Note that ADC readings below 96 are inaccurate; probably due to regulator
 * drop-out
 *
 * V = 0.0155*adc + 1
 */
uint8_t main_battery() {
   // TODO: implement this properly
   uint16_t adc = adc_read(7);
   uint16_t volts = ((adc*10)/64) + 10; // Volts*10
   if(adc < 384) volts=60; // clamp to 6V in non-linear range
   return volts;
}

/* read voltage of motor battery. Volts*10 */
/*
 * Measurement notes:
 *  10.02V: 159
 *   9.45V: 150
 *   8.93V: 141
 *   8.09V: 128
 *   6.95V: 110
 *   6.42V: 101
 *   5.99V: 94
 *
 * V = 0.015546*adc + 0.134
 * V = adc/64 + 0.134
 */
uint8_t motor_battery(){
   // TODO: implement this properly
   // this is likely to share A LOT of code with main_battery()
   uint16_t adc = adc_read(15);
   uint16_t volts = ((adc*10)/64) + 1;

   return volts;
}
