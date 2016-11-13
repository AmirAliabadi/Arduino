
#include <Servo.h> 

Servo esc1; 
Servo esc2;
Servo esc3;
Servo esc4;

#define MAX_SIGNAL 2000      // Simulate throttle at full
#define MIN_SIGNAL 1100     // Simulate throttle at min
#define MOTOR_PIN_A 3
#define MOTOR_PIN_C 6
#define MOTOR_PIN_B 5
#define MOTOR_PIN_D 10

#define THROTTLE_INPUT_PIN 0

#define P_PIN 1
#define I_PIN 2
#define D_PID 3

long last ;
long log_line = 0;

/*
1024 seemed to be the min value for motor to start
*/

void setup()
{
  Serial.begin(115200);


  Serial.println("attaching motors pins...");
  
  esc1.attach(MOTOR_PIN_A);
  esc2.attach(MOTOR_PIN_C);
  esc3.attach(MOTOR_PIN_B);
  esc4.attach(MOTOR_PIN_D);  

  delay(1000);

  Serial.println("attach motors to power...");
  
  //delay(5000);

  Serial.println("motors attached");

  Serial.println(sizeof(esc1));

  last = millis();
}

void loop()
{
  esc();
  //pid();
  //tune_up_down();
}

float test_value = 0;
void tune_up_down()
{
  float v = analogRead(THROTTLE_INPUT_PIN);
  //float tune = map(v, 0, 935, 0, 935) / 1000;
  //float tune =  (float)((int)((v / 10000000.0) * 100000.0 + .5))/100000.0;
  float tune = 0.0 ;
  if( v < 200 ) tune = -0.00001;
  else if (v > 700 ) tune = 0.00001;

  //tune = tune - 0.00005;

  if( millis() - last > 200 ) {
    test_value += tune;
    
    last = millis();
    Serial.print(v, 3);
    Serial.print(" ");

    Serial.print(test_value, 5);
    Serial.print(" ");
    
    Serial.println(tune, 5);
  }
  
}

void esc()
{
 
  float throt = map(analogRead(THROTTLE_INPUT_PIN),0,644,0,900);

  if( millis() - last > 200 ) {
    last = millis();
    Serial.println(MIN_SIGNAL+throt,4);
  }
  
  esc1.writeMicroseconds(MIN_SIGNAL+throt);  
  esc2.writeMicroseconds(MIN_SIGNAL+throt);  
  esc3.writeMicroseconds(MIN_SIGNAL+throt);  
  esc4.writeMicroseconds(MIN_SIGNAL+throt);  
  
}

void pid()
{
  float p = map(analogRead(P_PIN),0,644,0.0,100000.0)/20000.0;
  float i = map(analogRead(I_PIN),0,644,0.0,100000.0)/50000.0;
  float d = map(analogRead(D_PID),0,644,0.0,100000.0)/30000.0;

  if( millis() - last > 10 ) {
    last = millis();  
    Serial.print(log_line++);
    Serial.print("\t");
    
    Serial.print((int)(p*100.0+.5)/100.0,4);Serial.print("\t");
    Serial.print((int)(i*100.0+.5)/100.0,4);Serial.print("\t");
    Serial.print((int)(d*100.0+.5)/100.0,4);Serial.println("\t");
  }

}

