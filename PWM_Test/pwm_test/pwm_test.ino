//#include <SpacebrewYun.h>

//#include <Servo.h>
//#define _ESC_ Servo
//
#include "ESC.h"
#define _ESC_ ESC


int i = 0;
int foo = + 10;
int val = 0;
int last_start_tick = 0;
int duty_cycle_tick = 0;


#define MOTOR_PIN_A       3       // setup with 500hz PWM pins
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10 

_ESC_ esc_a;
//_ESC_ esc_b;
//_ESC_ esc_c;
//_ESC_ esc_d;

void setup() {
  Serial.begin(115200); 
  while (!Serial);
//  
//  pinMode(5, OUTPUT);  

  esc_a.attach(3);
//  esc_b.attach(9);
//  esc_c.attach(11);
//  esc_d.attach(10);  

//  esc_a.writeMicroseconds(1000);
//  esc_b.writeMicroseconds(1000);
//  esc_c.writeMicroseconds(1000);
//  esc_d.writeMicroseconds(1000);      
}

void loop() {
//  digitalWrite(5, HIGH);
//  delayMicroseconds(100); // Approximately 10% duty cycle @ 1KHz
//  digitalWrite(5, LOW);
//  delayMicroseconds(900);
//

    val = analogRead(3);    // read the input pin

    Serial.println(val);

    esc_a.writeMicroseconds(1000 + val * 1.5);
////    esc_b.writeMicroseconds(i);
////    esc_c.writeMicroseconds(i);
////    esc_d.writeMicroseconds(i); 
//  }
}

