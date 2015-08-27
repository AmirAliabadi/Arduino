
//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
float read_throttle()
{
  if( thrust < NEUTRAL_THRUST ) thrust += .05;
  if( thrust >= NEUTRAL_THRUST ) thrust = NEUTRAL_THRUST;
  return thrust;

  return map(analogRead(THROTTLE_PIN), 0.0, 668.0, 0.0, 234.0);
}

double read_kp()
{
  return 1.5;
  double foo = map(analogRead(Kp_PIN), 0.0, 644.0, 0.0, 10000.0);
  foo = foo / 1000.0;
  
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print("Kp:"); Serial.println(foo,2);
  }
  return foo;
}
double read_ki()
{
  return 0.02;
  double foo = map(analogRead(Ki_PIN), 0.0, 644.0, 0.0, 10000.0);
  foo = foo / 2000.0;
  
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print(" Ki:"); Serial.println(foo,2);
  }
  return foo;
}
double read_kd()
{
  return .75;
  
  double foo = map(analogRead(Kd_PIN), 0.0, 668.0, 0.0, 10000.0);
  foo = foo / 1000.0;
  
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print(" Kd:"); Serial.println(foo,2); Serial.println("\n");
  }
  return foo;
}
///////////////////////////////////////////////////////////////////////
