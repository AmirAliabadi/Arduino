#include <Servo.h> 

Servo esc1; 
Servo esc2;
Servo esc3;
Servo esc4;

#define MAX_SIGNAL 2000      // Simulate throttle at full
#define MIN_SIGNAL 1000     // Simulate throttle at min
#define MOTOR_PIN1 9

#define VELOCITY_INPUT_PIN 0

/*
1024 seemed to be the min value for motor to start
*/

void setup()
{
  Serial.begin(115200);
  
  esc1.attach(MOTOR_PIN1);
}

void loop()
{
  float pot = map(analogRead(VELOCITY_INPUT_PIN),0,668,0,500.0);
 
  Serial.println(MIN_SIGNAL+pot,4);
  
  esc1.writeMicroseconds(MIN_SIGNAL+pot);
  
  delay(100);
    
}
