#ifdef DEBUG 

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[36];    // us take the byte array
  float asFloat[9];    // sent from processing and
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
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  byte Direct_Reverse_Rate = -1;
  while(Serial.available() && index<40) //aa 26
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else if(index==2) Direct_Reverse_Rate = Serial.read();
    else if(index==3) selected_pot_tuning = Serial.read();
    else foo.asBytes[index-4] = Serial.read();
    index++;
  } 

  // if the information we got was in the correct format, 
  // read it into the system
  if(index==40  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    //setpoint[AC]=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      //aa output_ypr[AC]=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.

    /*
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    INPUT_STB_PID_P       = p;
    INPUT_STB_PID_I       = i;
    INPUT_STB_PID_D       = d;
    ac_pid.SetTunings(p, i, d);            //

    p = double(foo.asFloat[6]);           //
    i = double(foo.asFloat[7]);           //
    d = double(foo.asFloat[8]);           //
    INPUT_RAT_PID_P       = p;
    INPUT_RAT_PID_I       = i;
    INPUT_RAT_PID_D       = d;    
    ac_rat.SetTunings(p, i, d);            //  
    */  
    
    if(Auto_Man==0) 
    {
      ac_pid.SetMode(MANUAL);// * set the controller mode
      ac_rat.SetMode(MANUAL);// * set the controller mode
    }
    else 
    {
      ac_pid.SetMode(AUTOMATIC);             //
      ac_rat.SetMode(AUTOMATIC);             //      
    }
    
    if(Direct_Reverse==0) ac_pid.SetControllerDirection(DIRECT);// * set the controller Direction
    else ac_pid.SetControllerDirection(REVERSE);          //

    if(Direct_Reverse_Rate==0) ac_rat.SetControllerDirection(DIRECT);// * set the controller Direction
    else ac_rat.SetControllerDirection(REVERSE);          //    
  }
  Serial.flush();                         // * clear any random data from the serial buffer

  SerialSend();
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
// PID _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder
  
  Serial.print(selected_pot_tuning);//"PID ");

  Serial.print(INPUT_THRUST);  
  Serial.print(" ");
  
/////////////////////////////
// INPUTS
  Serial.print(setpoint[AC]);   
  Serial.print(" ");
  Serial.print(input_gyro[AC]);   
  Serial.print(" ");
  Serial.print(input_ypr[AC]);   
  Serial.print(" ");  
//
/////////////////////////////  

/////////////////////////////  
/// outputs
//  Serial.print(va - MIN_ESC_SIGNAL - INPUT_THRUST); //output_ypr[AC]);   
//  Serial.print(" ");  
//  Serial.print(vc - MIN_ESC_SIGNAL - INPUT_THRUST); //output_rate[AC]);   
//  Serial.print(" ");
  Serial.print(output_ypr[AC]);   
  Serial.print(" ");  
  Serial.print(output_rate[AC]);   
  Serial.print(" ");
/////////////////////////////  

  Serial.print(ac_pid.GetKp(), 3);   
  Serial.print(" ");
  Serial.print(ac_pid.GetKi(), 3);   
  Serial.print(" ");
  Serial.print(ac_pid.GetKd(), 3);   
  Serial.print(" ");
  
  Serial.print(ac_rat.GetKp(), 3);   
  Serial.print(" ");
  Serial.print(ac_rat.GetKi(), 3);   
  Serial.print(" ");
  Serial.print(ac_rat.GetKd(), 3);   
  Serial.print(" ");

  
  if(ac_pid.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(ac_pid.GetDirection()==DIRECT) Serial.print("Dir");
  else Serial.print("Rev");
  Serial.print(" ");
  if(ac_rat.GetDirection()==DIRECT) Serial.println("Dir");
  else Serial.println("Rev");  
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
