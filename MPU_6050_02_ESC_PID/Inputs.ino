
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
  return user_inputs.thrust;
}

float read_setpoint_ac()
{
  return user_inputs.setpoint.ac ;
}

double read_kp()
{
  return user_inputs.pid_ac[0].kp;    
}
double read_ki()
{
  return user_inputs.pid_ac[0].ki;  
}
double read_kd()
{
  return user_inputs.pid_ac[0].kd;
}
///////////////////////////////////////////////////////////////////////
