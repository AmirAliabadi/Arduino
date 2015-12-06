#ifdef DEBUG 

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[44];    // us take the byte array
  float asFloat[11];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-24: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  byte Direct_Reverse_Rate = -1;

  byte temp = -1;
  bool read_good = 1;
  while(Serial.available() && index<49) //aa 26
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else if(index==2) Direct_Reverse_Rate = Serial.read();
    else if(index==3){ temp = Serial.read(); if( temp < 0 or temp > 3){read_good = 0;} else{selected_pot_tuning = temp;} }
    else if(index==4){ temp = Serial.read(); if( temp < 0 or temp > 3){read_good = 0;} else{serial_data_mode = temp;} } 
    else foo.asBytes[index-5] = Serial.read();
    index++;
  } 

  if(!read_good) {
    Serial.flush();
    return;
  }
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==49) //  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    if( foo.asFloat[9] != foo.asFloat[10] ) return;
    if( serial_data_mode > 2 ) return;

    if( foo.asFloat[9] > INPUT_THRUST )
    { 
       if( foo.asFloat[9] > INPUT_THRUST + 50 ) INPUT_THRUST += 50;
       else INPUT_THRUST =  foo.asFloat[9];
    }
    else 
    {
      INPUT_THRUST = foo.asFloat[9] ;
    }
            
    if(Auto_Man==0) 
    {
      ac_pid.SetMode(MANUAL);
      ac_rat.SetMode(MANUAL);
      bd_pid.SetMode(MANUAL);
      bd_rat.SetMode(MANUAL);

      yw_pid.SetMode(MANUAL);
    }
    else 
    {
      ac_pid.SetMode(AUTOMATIC);
      ac_rat.SetMode(AUTOMATIC);  
      bd_pid.SetMode(AUTOMATIC);
      ac_rat.SetMode(AUTOMATIC);

      yw_pid.SetMode(AUTOMATIC);
    }

    
    if( serial_data_mode == 0 ) {
      if(Direct_Reverse==0) ac_pid.SetControllerDirection(DIRECT);
      else ac_pid.SetControllerDirection(REVERSE);
      
      if(Direct_Reverse_Rate==0) ac_rat.SetControllerDirection(DIRECT);
      else ac_rat.SetControllerDirection(REVERSE);  


      INPUT_STB_PID_P = foo.asFloat[3];
      INPUT_STB_PID_I = foo.asFloat[4];
      INPUT_STB_PID_D = foo.asFloat[5];
      INPUT_RAT_PID_P = foo.asFloat[6];
      INPUT_RAT_PID_I = foo.asFloat[7];
      INPUT_RAT_PID_D = foo.asFloat[8];

      
    } else if ( serial_data_mode == 1 ) {
      if(Direct_Reverse==0) bd_pid.SetControllerDirection(DIRECT);
      else bd_pid.SetControllerDirection(REVERSE);
      
      if(Direct_Reverse_Rate==0) bd_rat.SetControllerDirection(DIRECT);
      else bd_rat.SetControllerDirection(REVERSE); 


      INPUT_STB_PID_P = foo.asFloat[3];
      INPUT_STB_PID_I = foo.asFloat[4];
      INPUT_STB_PID_D = foo.asFloat[5];
      INPUT_RAT_PID_P = foo.asFloat[6];
      INPUT_RAT_PID_I = foo.asFloat[7];
      INPUT_RAT_PID_D = foo.asFloat[8];       
           
    } else if ( serial_data_mode == 2 ) {
      if(Direct_Reverse==0) yw_pid.SetControllerDirection(DIRECT);
      else yw_pid.SetControllerDirection(REVERSE);

      INPUT_YAW_PID_P = foo.asFloat[3];
      INPUT_YAW_PID_I = foo.asFloat[4]; 
      INPUT_YAW_PID_D = foo.asFloat[5];
            
    }
    
  }
  Serial.flush();                         // * clear any random data from the serial buffer

  if( serial_data_mode == 0 ) SerialSend_AC();
  else if( serial_data_mode == 1 ) SerialSend_BD();
  else if( serial_data_mode == 2 ) SerialSend_YAW();

  Serial.flush();                         // * clear any random data from the serial buffer

}

void SerialSend_YAW()
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder

  Serial.print(F("S "));  
  Serial.print(selected_pot_tuning);
  Serial.print("_");
  Serial.print(serial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));
  
/////////////////////////////
// INPUTS
  Serial.print(setpoint[YW]);   
  Serial.print(F(" "));
  Serial.print(input_gyro[YW]);   
  Serial.print(F(" "));
  Serial.print(input_ypr[YW]);   
  Serial.print(F(" "));  
//
/////////////////////////////  

/////////////////////////////  
/// outputs
//  Serial.print(va - MIN_ESC_SIGNAL - INPUT_THRUST); //output_ypr[AC]);   
//  Serial.print(" ");  
//  Serial.print(vc - MIN_ESC_SIGNAL - INPUT_THRUST); //output_rate[AC]);   
//  Serial.print(" ");
  Serial.print(output_ypr[YW]);   
  Serial.print(F(" "));  
  Serial.print(output_rate[YW]);   
  Serial.print(F(" "));
/////////////////////////////  

  Serial.print(yw_pid.GetKp(), 3);   
  Serial.print(F(" "));
  Serial.print(yw_pid.GetKi(), 3);   
  Serial.print(F(" "));
  Serial.print(yw_pid.GetKd(), 3);   
  Serial.print(F(" "));
  
  Serial.print(0.0,3); //yw_rat.GetKp(), 3);   
  Serial.print(F(" "));
  Serial.print(0.0,3); //yw_rat.GetKi(), 3);   
  Serial.print(F(" "));
  Serial.print(0.0,3); //yw_rat.GetKd(), 3);   
  Serial.print(F(" "));

  
  if(yw_pid.GetMode()==AUTOMATIC) Serial.print(F("Automatic"));
  else Serial.print(F("Manual"));  
  Serial.print(F(" "));
  if(yw_pid.GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev"));
  Serial.print(F(" "));
  Serial.print(F("Dir"));
  //if(yw_rat.GetDirection()==DIRECT) Serial.println(F("Dir"));
  //else Serial.println(F("Rev"));  

  Serial.print(F(" "));
  
  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));

  Serial.println(F("E"));  
}

void SerialSend_BD()
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder

  Serial.print(F("S "));  
  Serial.print(selected_pot_tuning);//"PID ");
  Serial.print("_");
  Serial.print(serial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));
  
/////////////////////////////
// INPUTS
  Serial.print(setpoint[BD]);   
  Serial.print(F(" "));
  Serial.print(input_gyro[BD]);   
  Serial.print(F(" "));
  Serial.print(input_ypr[BD]);   
  Serial.print(F(" "));  
//
/////////////////////////////  

/////////////////////////////  
/// outputs
//  Serial.print(va - MIN_ESC_SIGNAL - INPUT_THRUST); //output_ypr[AC]);   
//  Serial.print(" ");  
//  Serial.print(vc - MIN_ESC_SIGNAL - INPUT_THRUST); //output_rate[AC]);   
//  Serial.print(" ");
  Serial.print(output_ypr[BD]);   
  Serial.print(F(" "));  
  Serial.print(output_rate[BD]);   
  Serial.print(F(" "));
/////////////////////////////  

  Serial.print(bd_pid.GetKp(), 3);   
  Serial.print(F(" "));
  Serial.print(bd_pid.GetKi(), 3);   
  Serial.print(F(" "));
  Serial.print(bd_pid.GetKd(), 3);   
  Serial.print(F(" "));
  
  Serial.print(bd_rat.GetKp(), 3);   
  Serial.print(F(" "));
  Serial.print(bd_rat.GetKi(), 3);   
  Serial.print(F(" "));
  Serial.print(bd_rat.GetKd(), 3);   
  Serial.print(F(" "));

  
  if(bd_pid.GetMode()==AUTOMATIC) Serial.print(F("Automatic"));
  else Serial.print(F("Manual"));  
  Serial.print(F(" "));
  if(bd_pid.GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev"));
  Serial.print(F(" "));
  if(bd_rat.GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev")); 
  Serial.print(F(" "));
  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));

  Serial.println(F("E"));  
}

void SerialSend_AC()
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder

  Serial.print(F("S "));
  Serial.print(selected_pot_tuning);
  Serial.print("_");
  Serial.print(serial_data_mode);
  Serial.print(F(" "));

  Serial.print(INPUT_THRUST);  
  Serial.print(F(" "));
  
/////////////////////////////
// INPUTS
  Serial.print(setpoint[AC]);   
  Serial.print(F(" "));
  Serial.print(input_gyro[AC]);   
  Serial.print(F(" "));
  Serial.print(input_ypr[AC]);   
  Serial.print(F(" "));  
//
/////////////////////////////  

/////////////////////////////  
/// outputs
//  Serial.print(va - MIN_ESC_SIGNAL - INPUT_THRUST); //output_ypr[AC]);   
//  Serial.print(" ");  
//  Serial.print(vc - MIN_ESC_SIGNAL - INPUT_THRUST); //output_rate[AC]);   
//  Serial.print(" ");
  Serial.print(output_ypr[AC]);   
  Serial.print(F(" "));  
  Serial.print(output_rate[AC]);   
  Serial.print(F(" "));
/////////////////////////////  

  Serial.print(ac_pid.GetKp(), 3);   
  Serial.print(F(" "));
  Serial.print(ac_pid.GetKi(), 3);   
  Serial.print(F(" "));
  Serial.print(ac_pid.GetKd(), 3);   
  Serial.print(F(" "));
   
  Serial.print(ac_rat.GetKp(), 3);   
  Serial.print(F(" "));
  Serial.print(ac_rat.GetKi(), 3);   
  Serial.print(F(" "));
  Serial.print(ac_rat.GetKd(), 3);   
  Serial.print(F(" "));

  
  if(ac_pid.GetMode()==AUTOMATIC) Serial.print(F("Automatic"));
  else Serial.print(F("Manual"));  
  Serial.print(F(" "));
  if(ac_pid.GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev"));
  Serial.print(F(" "));
  if(ac_rat.GetDirection()==DIRECT) Serial.print(F("Dir"));
  else Serial.print(F("Rev"));  
  Serial.print(F(" "));
  
  Serial.print(va); Serial.print(F(" "));
  Serial.print(vb); Serial.print(F(" "));
  Serial.print(vc); Serial.print(F(" "));
  Serial.print(vd); Serial.print(F(" "));

  Serial.println(F("E"));
    
}


//
//void log_data()
//{
//  // line thrust  setpoint_yw setpoint_roll setpoint_pitch  input_yy  input_roll  input_pitch output_yw output_roll output_pitch  va  vc  vb  vd  ac_pid.pterm  ac_pid.iterm  ac_pid.dterm  kp  ki  kd
//  
//  Serial.print(log_line++);
//  Serial.print(F("\t"));
//  
//  Serial.print(INPUT_THRUST);
//  Serial.print(F("\t"));
///*
//  Serial.print(F("setpoint: "));
//
//  Serial.print(setpoint[YW], 1);
//  Serial.print(F("\t"));  
//  Serial.print(setpoint[AC], 1);
//  Serial.print(F("\t"));  
//  Serial.print(setpoint[BD], 1);
//  Serial.print(F("\tyw:"));  
//*/
///*
//  Serial.print(input_ypr[YW], 1);
//  Serial.print(F("\t"));
//*/
//  Serial.print(input_ypr[AC], 1);
//  Serial.print(F("\t"));
///*
//  Serial.print(input_ypr[BD], 1);
//  Serial.print(F("\t"));
//
//  Serial.print(input_gyro[YW], 1);
//  Serial.print(F("\t")); */
//  
//  Serial.print(input_gyro[AC], 1);
//  Serial.print(F("\t"));
//
//  /*
//  Serial.print(input_gyro[BD], 1);
//  Serial.print(F("\t"));  */
//
////  Serial.print(output_ypr[YW], 1);
////  Serial.print(F("\t"));  
//  
//  Serial.print(output_ypr[AC], 1);
//  Serial.print(F("\t"));  
////  Serial.print(output_ypr[BD], 1);
////  Serial.print(F("\t")); 
//
////  Serial.print(output_rate[AC], 1);
////  Serial.print(F("\t"));  
////  Serial.print(output_rate[BD], 1);
////  Serial.print(F("\t"));  
//
//  Serial.print(va);
//  Serial.print(F("\t"));
//  Serial.print(vc);
//  Serial.print(F("\t"));
///*
//  Serial.print(vb/100.0);
//  Serial.print(F("\t"));
//  Serial.print(vd/100.0);
//  Serial.print(F("\t")); */
//    /*
//
//  Serial.print(ac_pid.pterm, 1);
//  Serial.print(F("\t"));
//
//  Serial.print(ac_pid.iterm, 1);
//  Serial.print(F("\t"));
//  Serial.print(ac_pid.dterm, 1);
//  Serial.print(F("\t"));
//
//  Serial.print(bd_pid.pterm, 1);
//  Serial.print(F("\t"));
//  Serial.print(bd_pid.iterm, 1);
//  Serial.print(F("\t"));
//  Serial.print(bd_pid.dterm, 1);
//  Serial.print(F("\t"));
//*/
//  Serial.print(ac_pid.GetKp()*10, 3);  Serial.print(F("\t"));
////  Serial.print(ac_rat.GetKp()*10, 3);  Serial.print(F("\t"));
//
//  /*
//  Serial.print(ac_pid.GetKi()*10, 3);
//  Serial.print(F("\t")); */
//  Serial.print(ac_pid.GetKd()*10, 3);
//  Serial.print(F("\t"));
//  /*
//  Serial.print(bd_pid.GetKp()*10, 3);
//  Serial.print(F("\t"));
//  Serial.print(bd_pid.GetKi()*10, 3);
//  Serial.print(F("\t"));
//  Serial.print(bd_pid.GetKd()*10, 3);
//  */
//
////  Serial.print("\t");
////  Serial.print(mpu.getDLPFMode(), 4);
////  Serial.print("\t");
////  Serial.print(mpu.getDHPFMode(), 4);  
//
//  Serial.println();  
//  
//}

#endif
