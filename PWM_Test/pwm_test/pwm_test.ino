//#include <SpacebrewYun.h>

//#include <Servo.h>
//#define _ESC_ Servo
//#define IS_SERVO
//
#include "ESC.h"
#define _ESC_ ESC
#define IS_ESC

int i = 0;
int val = 0;
long last_start_tick = 0;
long duty_cycle_tick = 0;


#define MOTOR_PIN_A       3       // setup with 500hz PWM pins
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10 

#define USE_SERVO
//#define USE_ANALOGWRITE

#ifdef USE_SERVO
_ESC_ esc_a;
_ESC_ esc_b;
_ESC_ esc_c;
_ESC_ esc_d;
#endif

void setup() {
  Serial.begin(115200); 
  while (!Serial);

#ifdef USE_SERVO
  esc_a.attach(3);
  esc_b.attach(9);
  esc_c.attach(11);
  esc_d.attach(10);  
#endif

#ifdef USE_ANALOGWRITE
  analogWrite(MOTOR_PIN_A, 1);
  analogWrite(MOTOR_PIN_B, 1);
  analogWrite(MOTOR_PIN_C, 1);
  analogWrite(MOTOR_PIN_D, 1);
#endif

}

int next = 1;
int foo = 1;
long m;

void loop() {
  m = millis();

  if( (m - duty_cycle_tick) > 1 )
  {
    duty_cycle_tick = m;
    next = next + foo;

    do_it();
    
    if( next == 255 ) foo = foo *  -1;
    if( next == 1 ) foo = foo * -1;
  }
}

void do_it()
{
#ifdef USE_SERVO
#ifdef IS_SERVO
  esc_a.writeMicroseconds( next * 10);
  esc_b.writeMicroseconds( next * 10);
  esc_c.writeMicroseconds( next * 10);
  esc_d.writeMicroseconds( next * 10);
#endif
#ifdef IS_ESC
  esc_a.writeMicroseconds( next );
  esc_b.writeMicroseconds( next );
  esc_c.writeMicroseconds( next );
  esc_d.writeMicroseconds( next );
#endif  
#endif

#ifdef USE_ANALOGWRITE
    analogWrite(MOTOR_PIN_A, next);
    analogWrite(MOTOR_PIN_B, next);
    analogWrite(MOTOR_PIN_C, next);
    analogWrite(MOTOR_PIN_D, next); 
#endif  
}

