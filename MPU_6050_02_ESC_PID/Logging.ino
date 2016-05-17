#ifdef DEBUG 

/********************************************
 * Serial Communication functions / helpers
 ********************************************/

union _from_processing { 
  byte asBytes[8];    
  float asFloat[2];   
}                    
from_processing; 


void SerialReceive()
{
  Serial.readBytes( from_processing.asBytes, 8 );
  Serial.flush();

  int cmd = (int)from_processing.asFloat[0];
  if( cmd <= 16 ) {
    input_values[ cmd ] = from_processing.asFloat[1] ;
    update_pid_settings();
    
  } else {
    if(cmd == 100) {
      aserial_data_mode = (int)from_processing.asFloat[1];
            
    } else if (cmd == 101) {
      
    } else if (cmd == 102) {
      
    }
  }
}

void SerialSend(byte mode)
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder

  Serial.print(F("S "));
  Serial.print(selected_pot_tuning);
  Serial.print(F("_"));
  Serial.print(aserial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));
  
/////////////////////////////
// INPUTS
  Serial.print(setpoint[mode]);   
  Serial.print(F(" "));
  Serial.print(input_gyro[mode]);   
  Serial.print(F(" "));
  Serial.print(input_ypr[mode]);   
  Serial.print(F(" "));  
//
/////////////////////////////  

/////////////////////////////  
/// outputs
  Serial.print(output_ypr[mode]);   
  Serial.print(F(" "));  
  Serial.print(output_rate[mode]);   
  Serial.print(F(" "));
/////////////////////////////  

  Serial.print(pid_stable[mode].GetKp(), 4);   
  Serial.print(F(" "));
  Serial.print(pid_stable[mode].GetKi(), 4);   
  Serial.print(F(" "));
  Serial.print(pid_stable[mode].GetKd(), 4);   
  Serial.print(F(" "));
   
  Serial.print(pid_rate[mode].GetKp(), 4);   
  Serial.print(F(" "));
  Serial.print(pid_rate[mode].GetKi(), 4);   
  Serial.print(F(" "));
  Serial.print(pid_rate[mode].GetKd(), 4);   
  Serial.print(F(" "));

  
  if(pid_stable[mode].GetMode()==AUTOMATIC) Serial.print(F("Automatic"));
  else Serial.print(F("Manual"));  
  Serial.print(F(" "));
  if(pid_stable[mode].GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev"));
  Serial.print(F(" "));
  if(pid_rate[mode].GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev"));  
  Serial.print(F(" "));
  
  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));

  Serial.println(F("E"));
    
}

void log_data()
{
  if( aserial_data_mode == 0 ) SerialSend(AC);
  else if( aserial_data_mode == 1 ) SerialSend(BD);
  else if( aserial_data_mode == 2 ) SerialSend(YW); 
}

#endif
