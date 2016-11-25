
#define MOTOR_PIN_A       3       
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10 

//#define USE_SERVO
#define USE_ESC
//#define USE_ANALOGWRITE

#ifdef USE_ANALOGWRITE
#define PWM_INCREMENT 1
#define pwm_20_percent 200
#define pwm_10_percent 1
#endif

#ifdef USE_SERVO
//#include <Servo.h>
//#define _ESC_ Servo
#define SERVO_MODE "Servo "
#define PWM_INCREMENT 10
#define pwm_20_percent 2000
#define pwm_10_percent 1000
#endif

#ifdef USE_ESC 
#include "ESC.h"
#define _ESC_ ESC
#define SERVO_MODE "ESC "
#define PWM_INCREMENT 10
#define pwm_20_percent 2000
#define pwm_10_percent 1000
#endif

#ifdef SERVO_MODE
_ESC_ esc_a;
_ESC_ esc_b;
_ESC_ esc_c;
_ESC_ esc_d;
#endif

void setup() {
  Serial.begin(115200); 
  while (!Serial);
  
#ifdef SERVO_MODE
Serial.println(SERVO_MODE);

esc_a.attach(MOTOR_PIN_A);
esc_b.attach(MOTOR_PIN_B);
esc_c.attach(MOTOR_PIN_C);
esc_d.attach(MOTOR_PIN_D);  

esc_a.writeMicroseconds( pwm_10_percent );
esc_b.writeMicroseconds( pwm_10_percent );
esc_c.writeMicroseconds( pwm_10_percent );
esc_d.writeMicroseconds( pwm_10_percent );

#endif

#ifdef USE_ANALOGWRITE
  Serial.println("analogWrite Mode");
  analogWrite(MOTOR_PIN_A, 1);
  analogWrite(MOTOR_PIN_B, 1);
  analogWrite(MOTOR_PIN_C, 1);
  analogWrite(MOTOR_PIN_D, 1);
#endif

}

unsigned long tick;
unsigned long last_tick = 0;

int pwm_output = pwm_10_percent;
int pwm_increment = PWM_INCREMENT;
int top_hold = 100;
void loop() {
  tick = millis();

  if( (tick - last_tick) > 10 )
  {
    last_tick = tick;
    
    pwm_output = pwm_output + pwm_increment;

    do_it();
    
    if( pwm_output >= pwm_20_percent ) { pwm_increment = pwm_increment * -1; delay(2500); }
    if( pwm_output <= pwm_10_percent ) { pwm_increment = pwm_increment * -1; delay(2500); }
  }
}

void do_it()
{
  
#ifdef SERVO_MODE
  Serial.print(SERVO_MODE);
  Serial.println( pwm_output );

  esc_a.writeMicroseconds( pwm_output );
  esc_b.writeMicroseconds( pwm_output );
  esc_c.writeMicroseconds( pwm_output );
  esc_d.writeMicroseconds( pwm_output );
#endif

#ifdef USE_ANALOGWRITE
  Serial.print("analogWrite: ");
  Serial.println( pwm_output );
  analogWrite(MOTOR_PIN_A, pwm_output);
  analogWrite(MOTOR_PIN_B, pwm_output);
  analogWrite(MOTOR_PIN_C, pwm_output);
  analogWrite(MOTOR_PIN_D, pwm_output);   
#endif
  
}

