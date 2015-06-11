/* power.h
 *
 * Robot power management functions, including:
 *  5V main regulator enable/disable
 *  full power off
 *  battery monitoring
 *
 * Author: Austin Hendrix
 */

#ifndef POWER_H
#define POWER_H

/* enable 5V regulator */
void pwr_on();

/* disable 5V regulator */
void pwr_sleep();

/* full system shutdown. there is no going back! */
void pwr_off();

/* initialize ADCs for reading battries */
void battery_init();

/* read voltage of electronics battery. Volts*10 */
uint8_t main_battery();

/* read voltage of motor battery. Volts*10 */
uint8_t motor_battery();

#endif
