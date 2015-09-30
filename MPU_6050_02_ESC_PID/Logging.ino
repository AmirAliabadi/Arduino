#ifdef DEBUG 
void log_data()
{
  Serial.print(log_line++);
  Serial.print(F("\t"));
  
  Serial.print(thrust);
  Serial.print(F("\t"));

  for(int i=0;i <3; i++ ) {
    Serial.print(setpoint[i], 1);
    Serial.print(F("\t"));  
  }

  for(int i=0;i <3; i++ ) {
    Serial.print(input_ypr[i], 1);
    Serial.print(F("\t"));
  }
  
  for(int i=0;i <3; i++ ) {
    Serial.print(output_ypr[i], 1);
    Serial.print(F("\t"));  
  }

  Serial.print(va);
  Serial.print(F("\t"));
  Serial.print(vc);
  Serial.print(F("\t"));
  
  Serial.print(ac_pid.pterm, 4);
  Serial.print(F("\t"));
  Serial.print(ac_pid.iterm, 4);
  Serial.print(F("\t"));
  Serial.print(ac_pid.dterm, 4);
  Serial.print(F("\t"));
  
  Serial.print(ac_pid.GetKp(), 4);
  Serial.print(F("\t"));
  Serial.print(ac_pid.GetKi(), 4);
  Serial.print(F("\t"));
  Serial.println(ac_pid.GetKd(), 4);

//  Serial.print("\t");
//  Serial.print(mpu.getDLPFMode(), 4);
//  Serial.print("\t");
//  Serial.print(mpu.getDHPFMode(), 4);  
//  Serial.println();  
}

#endif
