
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

  }

  return 1;
}

//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
void read_throttle()
{
  thrust = constrain(input_values[0], MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming
}

void read_setpoint(int type)
{
  if( setpoint[type] != input_values[1] ) 
  {  
    setpoint_changed |= SETPOINT_CHANGED_AC;
    last_setpoint[type] = setpoint[type]; 
    setpoint[type] = input_values[1];
  }
}

void read_battery_voltage() {
  voltage = input_values[8];
}

void read_pid_tunings(int type)
{
  pid_xx_kp[type] = input_values[2];
  pid_xx_ki[type] = input_values[3];
  pid_xx_kd[type] = input_values[4];
}
///////////////////////////////////////////////////////////////////////
