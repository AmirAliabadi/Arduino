#ifdef USE_ESC

#include "ESC.h"
#define _ESC_ ESC

#define SERVO_MODE "ESC "
#define PWM_INCREMENT 10
#define PWM_00_PERCENT 1000
#define PWM_10_PERCENT 1090
#define PWM_20_PERCENT 2000

_ESC_ esc_a;
_ESC_ esc_b;
_ESC_ esc_c;
_ESC_ esc_d;


int get_pwm_increment()
{
  return PWM_INCREMENT;
}

int get_pwm_00_percent()
{
  return PWM_00_PERCENT; 
}


int get_pwm_10_percent()
{
  return PWM_10_PERCENT; 
}

int get_pwm_20_percent()
{
  return PWM_20_PERCENT; 
}

void pwm_setup() 
{
  Serial.println(SERVO_MODE);
  
  esc_a.attach(MOTOR_PIN_A);
  esc_b.attach(MOTOR_PIN_B);
  esc_c.attach(MOTOR_PIN_C);
  esc_d.attach(MOTOR_PIN_D);  
  
  esc_a.writeMicroseconds( pwm_10_percent );
  esc_b.writeMicroseconds( pwm_10_percent );
  esc_c.writeMicroseconds( pwm_10_percent );
  esc_d.writeMicroseconds( pwm_10_percent );
}

void do_it()
{
  Serial.print(SERVO_MODE);
  Serial.println( pwm_output );

  esc_a.writeMicroseconds( pwm_output );
  esc_b.writeMicroseconds( pwm_output );
  esc_c.writeMicroseconds( pwm_output );
  esc_d.writeMicroseconds( pwm_output );  
}

#endif

