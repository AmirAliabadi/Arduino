//take the string the arduino sends us and parse it
void serialEvent_floats(Serial myPort) {
  //char[] b = new char[]{0x3f, 0x9d, 0xf3, 0xb6};
  //byte[] b = new byte[]{0x3f, (0x9d), 0xf3, 0xb6};
  // float myfloatvalue = ByteBuffer.wrap(b).getFloat();
  // println(myfloatvalue);
// 0   0          72 66 50.00 
// 0   186        91 73 900000.00 
// 0   BA         5B 49 900000.00
// 0   FFFFFFBA   5B 49 900000.00
// B6  F3         9D 3F 1.23
  
  //String read = myPort.readStringUntil(10);
  //print(read);


  int size = 1;
  byte[] bytes = new byte[size * 4];
  float[] foo = new float[size];
  
  int byteCount = myPort.readBytes(bytes);
  
  if( byteCount == size*4 ) {
      for( int n=0; n<size; n+=4 ) {
        //byte[] b = new byte[] {bytes[n+3], bytes[n+2], bytes[n+1], bytes[n]};
        float myfloatvalue = ByteBuffer.wrap(bytes).getFloat();        
        
       float f1 = ByteBuffer.wrap(bytes).order(java.nio.ByteOrder.LITTLE_ENDIAN).getFloat();    
      
      //foo[n] = Float.intBitsToFloat( byteBuffer[n] ^ byteBuffer[n+1]<<8 ^ byteBuffer[n+2]<<16 ^ byteBuffer[n+3]<<24 );
      //float myfloatvalue = Float.intBitsToFloat( bytes[n+3] ^ bytes[n+2]<<8 ^ bytes[n+1]<<16 ^ bytes[n]<<24 );
      
      //print(foo[n]);
      print(myfloatvalue);
      print(" ");
    }
    println();
  } else {
    println("nope");
  }
  
}

void serialEvent(Serial myPort)
{
// S thrust _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder _ dir/inder E
// S 0_0 0.00 0.00 0.00 0.15 0.00 0.00 3.000 0.000 0.000 0.960 0.000 0.096 Manual Dir Dir 1100 1100 1100 1100 E

// HEADER
// t S selected.pot.tuning_serial.data.mode INPUT_THRUST SETPOINT inputgyro inputypr outputypr outputrate kp ki kd rKp rKi rKd A/M D/R rD/R va vb vc vd E
  
  String read = myPort.readStringUntil(10);
    
  if(outputFileName!="") output.print(str(millis())+ " " + read);
  String[] s = split(read, " ");
  
  if( trim(s[0]).equals("A") ) {
    parseA(s);
  } else if (trim(s[0]).equals("B")) {
    parseB(s);
  } else {
    print(read);
  }
}

void parseA(String[] s ) {
  // A 0_0 0.00 0.00 2.4900 0.0000 0.0000 0.3760 0.0000 0.0040 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 E
  // 0 1   2    3    4      5      6      7      8      9      10     11     12     13     14     15     16
  
  int expected_size = 17; 
  if (s.length == expected_size)
  {
    if(! trim(s[expected_size-1]).equals("E") ) {print(s.length); return;}

    Output_angle = float(trim(s[2]));  // stable pid output (current angle - setpoint) => desired acceleration
    Output_gyro = float(trim(s[3]));   // this is the rate pid output (desired acceleration - current acceleration) => motor inputs
    
    kp = float(trim(s[4]));
    ki = float(trim(s[5]));
    kd = float(trim(s[6]));
    krp = float(trim(s[7]));
    kri = float(trim(s[8]));
    krd = float(trim(s[9]));

    pterm_stable = float(trim(s[10]));
    iterm_stable = float(trim(s[11]));
    dterm_stable = float(trim(s[12]));
    
    pterm_rate  = float(trim(s[13]));
    iterm_rate  = float(trim(s[14]));
    dterm_rate  = float(trim(s[15]));

    PTermRateLabel.setValue(Float.toString(pterm_rate) );
    ITermRateLabel.setValue(Float.toString(iterm_rate) );
    DTermRateLabel.setValue(Float.toString(dterm_rate) );
    
    
    AngleOutLabel.setValue(Float.toString(Output_angle)); 
    GyroOutLabel.setValue(Float.toString(Output_gyro));    

    PLabel.setValue(Float.toString(kp));
    ILabel.setValue(Float.toString(ki)); 
    DLabel.setValue(Float.toString(kd)); 
    
    PrLabel.setValue(Float.toString(krp));  
    IrLabel.setValue(Float.toString(kri));
    DrLabel.setValue(Float.toString(krd)); 
    
    if(justSent)                     
    {                                
      justSent=false;
    } 

    if(!madeContact) madeContact=true;    

  }
}

void parseB(String[] s ) {
  // B 0_0 0.00 0.00 0.00 -34.45 A D D 1000 1000 1000 1000 0.80 E
  // 0 1   2    3    4    5      6 7 8 9    10   11   12   13   14
  
  int expected_size = 16; 
  
  if (s.length == expected_size)
  {
    if(! trim(s[expected_size-1]).equals("E") ) {print(s.length); return;}

    Input_Thrust   = float(trim(s[2])); 
    Input_Setpoint = float(trim(s[3]));
    Input_gyro     = float(trim(s[4]));
    Input_angle    = float(trim(s[5]));
    
    SPField.setValue( Float.toString(Input_Setpoint) );
    
    cur_throttle = (int)Input_Thrust; 
    InputThrustLabel.setValue( Float.toString(Input_Thrust) );
    throttle_slider.setValue( Input_Thrust );
    
    InputSetpointLabel.setValue(trim(s[3]));
    InputGyroLabel.setValue(trim(s[4])); 
    InputAngleLabel.setValue(trim(s[5]));
    
    DRCurrent.setValue(trim(s[7]));
    DRrCurrent.setValue(trim(s[8]));
    
    va = float(trim(s[ 9]));
    vb = float(trim(s[10]));
    vc = float(trim(s[11]));
    vd = float(trim(s[12]));
    
    alpha = float(trim(s[13]));
    AlphaLable.setValue(Float.toString(alpha));
    
    pid_refresh_rate = float(trim(s[14]));
    PidRefreshIntervalLabel.setValue(Float.toString(pid_refresh_rate));    
    
    controlP5.getController("va").setValue(va);
    controlP5.getController("vb").setValue(vb);
    controlP5.getController("vc").setValue(vc);
    controlP5.getController("vd").setValue(vd);
    
    if( va > vc )
    {
      int c = (int)abs(va-vc)*7;
      controlP5.getController("va").setColorForeground(color(c, 0, 255));
      controlP5.getController("vc").setColorForeground(color(0, 0, 255-c));    
    }
    else
    {
      int c = (int)abs(va-vc)*7;
      controlP5.getController("va").setColorForeground(color(0, 0, 255-c));
      controlP5.getController("vc").setColorForeground(color(c, 0, 255));  
    }
    
    if( vb > vd )
    {
      int c = (int)abs(vb-vd)*7;      
      controlP5.getController("vb").setColorForeground(color(c, 0, 255));
      controlP5.getController("vd").setColorForeground(color(0, 0, 255-c));    
    }
    else
    {
      int c = (int)abs(vb-vd)*7;
      controlP5.getController("vb").setColorForeground(color(0, 0, 255-c));
      controlP5.getController("vd").setColorForeground(color(c, 0, 255));  
    }  
    
    if(justSent)                     
    {                                
      justSent=false;
    } 

    if(!madeContact) madeContact=true;    
  }
}