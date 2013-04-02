/* encoder.c
 *
 * Interrupt-driven encoder driver for Dagny
 *
 * Author: Austin Hendrix
 *
 * The front and rear gear sensors are attached to PCINT20(PK4) and PCINT21(PK5)
 *  respsectively
 *
 */

#include <avr/interrupt.h>

#define Q1 (1 << PCINT20)
#define Q2 (1 << PCINT21)

//uint8_t q1;
//uint8_t q2;
uint8_t old;

//volatile int16_t qspeed; /* quaderature encoder speed */
volatile int16_t qcount; /* quaderature encoder 1/4 turn count */

void encoder_init() {
   qcount = 0;

   // set up input pins
   DDRK &= ~( Q1 | Q2 );
   // set pull-ups on inputs
   PORTK |= ( Q1 | Q2 );
   
   old = PINK & (Q1 | Q2);;

   // Enable pin-change interrupt
   PCMSK2 = Q1 | Q2;
   PCICR |= (1 << PCIE2);
}

ISR(PCINT2_vect) {
   // read input
   uint8_t input = PINK & (Q1 | Q2);

   uint8_t diff = input ^ old;
   uint8_t q1 = (input >> PCINT20) & 0x1;
   uint8_t q2 = (input >> PCINT21) & 0x1;

   /* read the quaderature encoder on the drive gear to get better
      direction and speed data */
   if( diff & Q1 ) { // Q1 change
      if( q1 == q2 ) { // q1 == q2
         // turning forward
         qcount++;
      } else {
         // turning backward
         qcount--;
      }
   } else if( diff & Q2 ) { // Q2 change
      if( q1 == q2 ) { // q1 == q2
         // turning backward
         qcount--;
      } else {
         // turning forward
         qcount++;
      }
   }
   old = input;
}
