////////////////////////////////////////////////////////////////
// throttle
#define THROTTLE_PIN 0

////////////////////////////////////////////////////////////////
// PID Tunning with a POT
#define Kp_PIN 1
#define Ki_PIN 2
#define Kd_PIN 3


//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
float read_throttle()
{
  if( thrust < NEUTRAL_THRUST ) thrust += .10;
  if( thrust >= NEUTRAL_THRUST ) thrust = NEUTRAL_THRUST;
  return thrust;

  return map(analogRead(THROTTLE_PIN), 0.0, 668.0, 0.0, 234.0);
}

double read_kp()
{
  return 2.0;
  double foo = map(analogRead(Kp_PIN), 0.0, 668.0, 0.0, 10000.0);

  foo = foo / 4000.0;
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print("Kp:"); Serial.print(foo,2);
  }
  return foo;
}
double read_ki()
{
  return 0.2;
  double foo = map(analogRead(Ki_PIN), 0.0, 668.0, 0.0, 10000.0);
  
  foo = foo / 2000.0;
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print(" Ki:"); Serial.print(foo,2);
  }
  return foo;
}
double read_kd()
{
  return 0.75;
  double foo = map(analogRead(Kd_PIN), 0.0, 668.0, 0.0, 10000.0);
  
  foo = foo / 8000.0;
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print(" Kd:"); Serial.print(foo,2); Serial.println("");
  }
  return foo;
}
///////////////////////////////////////////////////////////////////////
