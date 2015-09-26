
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

  Serial.print("#### \t");
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

    Serial.print(pidVector[k-1]);
    Serial.print("\t");
  }
  Serial.println("####");
  return 1;
}

//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
float read_throttle()
{
  return thrust;
}

float read_setpoint_ac()
{
  return setpoint_ac ;
}

double read_kp()
{
  return pid_ac_kp[0];    
}
double read_ki()
{
  return pid_ac_ki[0];  
}
double read_kd()
{
  return pid_ac_kd[0];
}
///////////////////////////////////////////////////////////////////////
