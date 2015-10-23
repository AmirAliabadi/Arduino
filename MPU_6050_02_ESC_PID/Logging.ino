#ifdef DEBUG 
void log_data()
{
  // line thrust  setpoint_yw setpoint_roll setpoint_pitch  input_yy  input_roll  input_pitch output_yw output_roll output_pitch  va  vc  vb  vd  ac_pid.pterm  ac_pid.iterm  ac_pid.dterm  kp  ki  kd
  
  Serial.print(log_line++);
  Serial.print(F("\t"));
  
  Serial.print(INPUT_THRUST);
  Serial.print(F("\t"));
/*
  Serial.print(F("setpoint: "));

  Serial.print(setpoint[YW], 1);
  Serial.print(F("\t"));  
  Serial.print(setpoint[AC], 1);
  Serial.print(F("\t"));  
  Serial.print(setpoint[BD], 1);
  Serial.print(F("\tyw:"));  
*/

  Serial.print(input_ypr[YW], 1);
  Serial.print(F("\t"));
  Serial.print(input_ypr[AC], 1);
  Serial.print(F("\t"));
  Serial.print(input_ypr[BD], 1);
  Serial.print(F("\t"));

  Serial.print(input_gyro[YW], 1);
  Serial.print(F("\t"));
  Serial.print(input_gyro[AC], 1);
  Serial.print(F("\t"));
  Serial.print(input_gyro[BD], 1);
  Serial.print(F("\t"));  

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

//  Serial.print(ax);
//  Serial.print(F("\t"));
//  Serial.print(ay);
//  Serial.print(F("\t"));
//  Serial.print(az);
//  Serial.print(F("\t"));
//  Serial.print(gx);
//  Serial.print(F("\t"));
//  Serial.print(gy);
//  Serial.print(F("\t"));
//  Serial.print(gz);
//  Serial.print(F("\t")); 

  Serial.print(output_ypr[YW], 1);
  Serial.print(F("\t"));  
  
  Serial.print(output_ypr[AC], 1);
  Serial.print(F("\t"));  
  Serial.print(output_ypr[BD], 1);
  Serial.print(F("\t")); 

  Serial.print(output_rate[AC], 1);
  Serial.print(F("\t"));  
  Serial.print(output_rate[BD], 1);
  Serial.print(F("\t"));  

  Serial.print(va/100.0);
  Serial.print(F("\t"));
  Serial.print(vc/100.0);
  Serial.print(F("\t"));
  Serial.print(vb/100.0);
  Serial.print(F("\t"));
  Serial.print(vd/100.0);
  Serial.print(F("\t"));

  /*
  Serial.print(ac_pid.pterm, 1);
  Serial.print(F("\t"));
  Serial.print(ac_pid.iterm, 1);
  Serial.print(F("\t"));
  Serial.print(ac_pid.dterm, 1);
  Serial.print(F("\t"));

  Serial.print(bd_pid.pterm, 1);
  Serial.print(F("\t"));
  Serial.print(bd_pid.iterm, 1);
  Serial.print(F("\t"));
  Serial.print(bd_pid.dterm, 1);
  Serial.print(F("\t"));

  Serial.print(ac_pid.GetKp()*10, 3);
  Serial.print(F("\t"));
  Serial.print(ac_pid.GetKi()*10, 3);
  Serial.print(F("\t"));
  Serial.print(ac_pid.GetKd()*10, 3);
  Serial.print(F("\t"));
  
  Serial.print(bd_pid.GetKp()*10, 3);
  Serial.print(F("\t"));
  Serial.print(bd_pid.GetKi()*10, 3);
  Serial.print(F("\t"));
  Serial.print(bd_pid.GetKd()*10, 3);
  */

//  Serial.print("\t");
//  Serial.print(mpu.getDLPFMode(), 4);
//  Serial.print("\t");
//  Serial.print(mpu.getDHPFMode(), 4);  

  Serial.println();  
}

#endif
