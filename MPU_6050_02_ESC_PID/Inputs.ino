
boolean readCsvToVector(float* pidVector) 
{
  byte len;
  if (Serial.available() <= 0)
    return 0;
   
  len = Serial.available();
  
  char stream[80];
  char number[10];
  
  // Read the message
  for (byte i = 0; i < len; ++i) {
    stream[i] = Serial.read();
  }
  
  byte i = 0, j = 0, k = 0;

  Serial.print("#\t");
  while ( i < len ) {
    while (stream[i] != ',' && i < len) {
      number[j] = stream[i];
      ++i;
      ++j;
    }
    ++i;
    number[j] = '\0';
    j = 0;
    pidVector[k++] = atof(number);

    Serial.print(pidVector[k-1],4);
    Serial.print("\t");
  }
  Serial.println("");
  return 1;
}

//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
float read_throttle()
{
  if(esc_ready)
  {
    return constrain(input_values[0], MIN_INPUT_THRUST, MAX_INPUT_THRUST);
  }
  return MIN_INPUT_THRUST;
  
  //#define MAX_INPUT_THRUST  1400    // safety setting while testing.
  //#define MIN_THRUST        1130    // motor is off below this value
  //float foo = map(analogRead(THROTTLE_PIN), 0.0, 668.0, 0.0, 400.0);
  //return (float)( (int) (((foo * 10.0)+.5)/10.0));
}

float read_setpoint_ac()
{
  return input_values[1];
  
  float foo = map(analogRead(THROTTLE_PIN), 0.0, 668.0, 0.0, 400.0);

  return (float)( (int) (((foo * 10.0)+.5)/10.0));
}

double read_kp()
{
  return input_values[2];
  
  double foo = map(analogRead(Kp_PIN), 0.0, 644.0, 0.0, 10000.0);
  foo = foo / 1000.0;
  foo = (double)( (int)((foo * 100.0)+.5) / 100.0 );
  return foo;
}
double read_ki()
{
  return input_values[3];
  
  double foo = map(analogRead(Ki_PIN), 0.0, 644.0, 0.0, 10000.0);
  foo = foo / 2000.0;
  foo = (double)( (int)((foo * 100.0)+.5) / 100.0 );
  return foo;
}
double read_kd()
{
  return input_values[4];
  
  double foo = map(analogRead(Kd_PIN), 0.0, 668.0, 0.0, 10000.0);
  foo = foo / 1000.0;
  foo = (double)( (int)((foo * 100.0)+.5) / 100.0 );  
  return foo;
}
///////////////////////////////////////////////////////////////////////
