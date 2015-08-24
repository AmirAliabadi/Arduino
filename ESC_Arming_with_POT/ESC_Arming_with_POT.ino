#include <Servo.h> 

Servo esc1; 
Servo esc2;
Servo esc3;
Servo esc4;

#define MAX_SIGNAL 2000      // Simulate throttle at full
#define MIN_SIGNAL 1000     // Simulate throttle at min
#define MOTOR_PIN1 9

#define VELOCITY_INPUT_PIN 0
#define P_PIN 0
#define I_PIN 1
#define D_PID 2

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

  float p = map(analogRead(P_PIN),0,644,0,10000.0)/1000.0;
  float i = map(analogRead(I_PIN),0,644,0,10000.0)/1000.0;
  float d = map(analogRead(D_PID),0,644,0,10000.0)/1000.0;

  Serial.print((int)(p*10.0+.5)/10.0);Serial.print("\t");
  Serial.print((int)(i*10.0+.5)/10.0);Serial.print("\t");
  Serial.print((int)(d*10.0+.5)/10.0);Serial.println("\t");
 
  //Serial.println(MIN_SIGNAL+pot,4);
  
  //esc1.writeMicroseconds(MIN_SIGNAL+pot);
  
  delay(100);
    
}
