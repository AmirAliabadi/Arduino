void SerialSend_Minimal(byte mode)
{
  // M selected_pot_tuning_aserial_data_mode INPUT_THRUST va vb vc vd current_attitude current_rate attitude_correction rate_correction stable_p stable_i stable_d rate_p rate_i rate_d alpha E
  // M 0_0 328.00 0 0 0 0 0.99 0.00 nan 0.00 0.9990 0.0000 0.1000 0.0000 0.0000 0.0000 0.44 E

  
  Serial.print(F("M "));
  Serial.print(selected_pot_tuning);
  Serial.print(F("_"));
  Serial.print(aserial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));

  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));  

  Serial.print(current_attitude[mode]);     Serial.print(F(" "));  // this is the actual angle being read by the MPU    
  Serial.print(current_rate[mode]);         Serial.print(F(" "));  // this is the actual acceleration being read by the MPU

  Serial.print(attitude_correction[mode]);    Serial.print(F(" "));  // this is the stable pid output (measured angle - setpoint) => desired acceleration
  Serial.print(rate_correction[mode]);        Serial.print(F(" "));  // this is the rate pid output (desired acceleration - current acceleration) => motor output

  if( mode == 0x0 )
  {
    Serial.print(INPUT_YAW_PID_P, 4);   Serial.print(F(" "));
    Serial.print(INPUT_YAW_PID_I, 4);   Serial.print(F(" "));
    Serial.print(INPUT_YAW_PID_D, 4);   Serial.print(F(" "));
  
  #ifdef CASCADE_PIDS    
    Serial.print(INPUT_YAW_RATE_PID_P, 4);   Serial.print(F(" "));
    Serial.print(INPUT_YAW_RATE_PID_I, 4);   Serial.print(F(" "));
    Serial.print(INPUT_YAW_RATE_PID_D, 4);   Serial.print(F(" "));
  #else
    Serial.print(0.0000, 4);   Serial.print(F(" "));
    Serial.print(0.0000, 4);   Serial.print(F(" "));
    Serial.print(0.0000, 4);   Serial.print(F(" "));
  #endif    
  } else {
    Serial.print(INPUT_STB_PID_P, 4);   Serial.print(F(" "));
    Serial.print(INPUT_STB_PID_I, 4);   Serial.print(F(" "));
    Serial.print(INPUT_STB_PID_D, 4);   Serial.print(F(" "));
  
  #ifdef CASCADE_PIDS   
    Serial.print(INPUT_RAT_PID_P, 4);   Serial.print(F(" "));
    Serial.print(INPUT_RAT_PID_I, 4);   Serial.print(F(" "));
    Serial.print(INPUT_RAT_PID_D, 4);   Serial.print(F(" "));
  #else
    Serial.print(INPUT_RAT_PID_P, 4);   Serial.print(F(" "));
    Serial.print(INPUT_RAT_PID_I, 4);   Serial.print(F(" "));
    Serial.print(INPUT_RAT_PID_D, 4);   Serial.print(F(" "));
  #endif
  }

  Serial.print(alpha); Serial.println(F(" E"));  
}

void SerialSend_Console_Out(byte mode)
{
  Serial.print(F("M "));

  Serial.print(INPUT_THRUST); Serial.print(F(" "));

  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));  

  Serial.print(current_attitude[mode]);     Serial.print(F(" "));  // this is the actual angle being read by the MPU    
  Serial.print(current_rate[mode]);         Serial.print(F(" "));  // this is the actual acceleration being read by the MPU

  Serial.print(attitude_correction[mode]);    Serial.print(F(" "));  // this is the stable pid output (measured angle - setpoint) => desired acceleration
  Serial.print(rate_correction[mode]);        Serial.print(F(" "));  // this is the rate pid output (desired acceleration - current acceleration) => motor output

  Serial.print(INPUT_STB_PID_P, 4);   Serial.print(F(" "));
  Serial.print(INPUT_STB_PID_I, 4);   Serial.print(F(" "));
  Serial.print(INPUT_STB_PID_D, 4);   Serial.print(F(" "));
  
  Serial.print(INPUT_RAT_PID_P, 4);   Serial.print(F(" "));
  Serial.print(INPUT_RAT_PID_I, 4);   Serial.print(F(" "));
  Serial.print(INPUT_RAT_PID_D, 4);   Serial.print(F(" "));

  Serial.print(alpha); 
  
  Serial.println(F(" E"));  
}


void log_data()
{
  if( aserial_data_mode == 0 ) SerialSend_Console_Out(AC);
  else if( aserial_data_mode == 1 ) SerialSend_Console_Out(BD);
  else if( aserial_data_mode == 2 ) SerialSend_Console_Out(YAW); 
}

