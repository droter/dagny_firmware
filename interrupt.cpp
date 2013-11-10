/*
 * New interrupt-based wheel and speed management; replaces the RTOS
 */

#include <avr/interrupt.h>
#include <math.h>

extern "C" {
#include "drivers/serial.h"
#include "motor.h"
#include "drivers/bump.h"
#include "estop.h"
}

#include "steer.h"
#include "imu.h"
#include "publish.h"
#include "twist.h"

#define abs(x) ((x)>0?(x):-(x))

uint32_t ticks = 0;

//uint8_t input, input_old;

#define L 0x80
#define R 0x40

volatile int16_t qspeed; /* quaderature encoder speed */
extern volatile int16_t qcount; /* quaderature encoder 1/4 turn count */

float x, y, yaw; /* position */
int16_t old_qcount; /* for updating odometry output */

#define DIV 256
#define MULT_START (DIV*4)

// speed management variables
volatile int16_t power = 0; 
volatile int16_t target_speed;

int16_t speed = 0;
int16_t mult = MULT_START;
int16_t e = 0; // error

// odometry transmission variables
volatile uint16_t odom_sz = 0;
volatile int8_t steer;
// (5 floats + overhead)*2 = 64
Publisher<96> odom('O');

// 0.03 meters per tick
//  TODO: update for increased encoder resolution
#define Q_SCALE (0.032 / 4.0)

uint16_t estop_cnt = 0;

/* set up interrupt handling */
void interrupt_init(void) {
   estop_init();

   // set motor pins as input
   DDRC &= ~( L | R );
   // set pull-ups on inputs
   PORTC |= ( L | R );

   // set up timer interrupts
   // fast PWM mode; interrupt and reset when counter equals OCR0A
   // prescalar 64
   TCCR0A = (1 << WGM01 | 1 << WGM00);
   TCCR0B = (1 << WGM02 | 1 << CS02);
   // interrupt on "overflow" (counter match)
   TIMSK0 = (1 << TOIE0);
   OCR0A  = (125) - 1; // 500 Hz
}

/* interrupt routine */
/* TIMER0 OVF */
ISR(TIMER0_OVF_vect) {
   ticks++;

   // enable nested interrupts
   sei();

   // speed management; run at 10Hz
   const static int16_t Kp = DIV/16; // proportional constant
   //if( ticks % 100 == 0 ) {
   // 10Hz = 500Hz / 50
   if( ticks % 50 == 0 ) {
      qspeed = (qcount - old_qcount);
      // E-stop
      if( estop() ) {
         estop_cnt = 30;
      }

      if( estop_cnt == 0 ) {
         //led_on();
         // reflex: stop if we bump into something
         if( target_speed > 0 && bump() ) {
            power = 0;
            mult = MULT_START;
         } else {
            speed = qspeed;

            e = target_speed - speed; 

            if( target_speed != 0 ) {
               e = Kp * e / target_speed;
               if( e > DIV ) e = DIV;
               if( e < -DIV ) e = -DIV;
               mult += e;
            } else {
               mult = MULT_START;
            }

            if( mult < 1 ) mult = 1;
            if( abs(mult*target_speed) > 100*DIV ) 
               mult = 100*DIV/abs(target_speed);

            power = mult * (double)target_speed;
         }
      } else {
         --estop_cnt;
         //led_off();
         power = 0;
      }

      // output
      motor_speed(power/DIV);

      // odometry
      double r = steer2radius(steer);

      // qspeed in ticks/sec
      float speed = qspeed * Q_SCALE * 10.0;

      // if we've moved, update position
      if( old_qcount != qcount ) {
         float d = (qcount - old_qcount) * Q_SCALE;
         float dx, dy, dt;
         if( steer == 0 ) {
            dx = d * cos(yaw);
            dy = d * sin(yaw);
            dt = 0.0;
         } else {
            dt = d / r;
            float theta_c1;
            float theta_c2;
            if( steer > 0 ) {
               // turning right
               theta_c1 = yaw + M_PI/2;
            } else {
               // turning left
               dt = -dt;
               theta_c1 = yaw - M_PI/2;
            }
            theta_c2 = theta_c1 - dt;

            dx = r * (cos(theta_c2) - cos(theta_c1));
            dy = r * (sin(theta_c2) - sin(theta_c1));
         }

         x += dx;
         y += dy;
         yaw -= dt; // TODO: figure out why this sign is flipped

         old_qcount = qcount;
      }

      if(odom.reset() ) {
         odom.append(speed); // linear speed
         if( steer == 0 ) {
            odom.append(0.0f);
         } else {
            odom.append((float)(speed / r)); // angular speed
         }
         // odom position in odom frame
         odom.append(x);
         odom.append(y);

         //extern Twist imu_state;
         //yaw = (yaw + imu_state.angular.z) / 2.0;
         //yaw = imu_state.angular.z;
         odom.append(yaw);
         odom.append(bump());
         odom.append(qcount);
         odom.append(steer);
         // odom: total of 5 floats; 4*5 = 20 bytes
         odom.finish();
      }
   }

   // IMU and GPS loop; run at 20Hz.
   // run at a time when the odometry calculations aren't running
   //if( ticks % 50 == 24 ) {
   // 20 Hz = 500Hz / 25.0
   if( ticks % 25 == 12 ) {
      imu_read(); // read the IMU
   }

   if( ticks == 5000 ) {
      ticks = 0;
   }
}
