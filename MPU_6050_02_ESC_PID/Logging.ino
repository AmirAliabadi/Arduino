#ifdef DEBUG 
void log_data()
{
  Serial.print(log_line++);
  Serial.print(F("\t"));
  
  Serial.print(thrust);
  Serial.print(F("\t"));

  for(int i=2;i <3; i++ ) {
    Serial.print(setpoint[i], 1);
    Serial.print(F("\t"));  
  }

  for(int i=2;i <3; i++ ) {
    Serial.print(input_ypr[i], 1);
    Serial.print(F("\t"));
  }
/*
    Serial.print((aaReal.x));
    Serial.print(F("\t"));
    Serial.print((aaReal.y));
    Serial.print(F("\t"));
    Serial.print((aaReal.z));  
    Serial.print(F("\t"));
*/    

//  for(int i=0;i <3; i++ ) {
//    Serial.print(gyro1[i], 1);
//    Serial.print(F("\t"));
//  }

  Serial.print(ax);
  Serial.print(F("\t"));
//  Serial.print(ay);
//  Serial.print(F("\t"));
//  Serial.print(az);
//  Serial.print(F("\t"));
  Serial.print(gx);
  Serial.print(F("\t"));
//  Serial.print(gy);
//  Serial.print(F("\t"));
//  Serial.print(gz);
//  Serial.print(F("\t")); 

  for(int i=2;i <3; i++ ) {
    Serial.print(output_ypr[i], 1);
    Serial.print(F("\t"));  
  }

/*
  Serial.print(va);
  Serial.print(F("\t"));
  Serial.print(vc);
  Serial.print(F("\t"));
*/  
  
  Serial.print(ac_pid.pterm, 1);
  Serial.print(F("\t"));
  Serial.print(ac_pid.iterm, 1);
  Serial.print(F("\t"));
  Serial.print(ac_pid.dterm, 1);
  Serial.print(F("\t"));
  Serial.print(F("\t"));

  Serial.print(ac_pid.GetKp()*10, 3);
  Serial.print(F("\t"));
  Serial.print(ac_pid.GetKi()*10, 3);
  Serial.print(F("\t"));
  Serial.print(ac_pid.GetKd()*10, 3);

//  Serial.print("\t");
//  Serial.print(mpu.getDLPFMode(), 4);
//  Serial.print("\t");
//  Serial.print(mpu.getDHPFMode(), 4);  

  Serial.println();  
}

#endif
