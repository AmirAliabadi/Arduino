//take the string the arduino sends us and parse it
void serialEvent(Serial myPort)
{
// S thrust _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder _ dir/inder E
// S 0_0 0.00 0.00 0.00 0.15 0.00 0.00 3.000 0.000 0.000 0.960 0.000 0.096 Manual Dir Dir 1100 1100 1100 1100 E

// HEADER
// t S selected.pot.tuning_serial.data.mode INPUT_THRUST SETPOINT inputgyro inputypr outputypr outputrate kp ki kd rKp rKi rKd A/M D/R rD/R va vb vc vd E
  
  String read = myPort.readStringUntil(10);
    
  if(outputFileName!="") output.print(str(millis())+ " " + read);
  String[] s = split(read, " ");
  
  if (s.length == 23)
  {
    if(! trim(s[0]).equals("S") )  {print(read); return;}
    if(! trim(s[22]).equals("E") ) {print(read); return;}
    
    print(read);
    
    Input_Thrust = float(trim(s[2])); 
    Input_Setpoint = float(trim(s[3]));
    Input_gyro = float(trim(s[4]));
    Input_angle = float(trim(s[5]));
    
    Output_angle = float(trim(s[6]));
    Output_gyro = float(trim(s[7]));    
    
    cur_throttle = (int)float(trim(s[2]));
    InputThrustLabel.setValue(trim(s[2]));
    throttle_slider.setValue( float(trim(s[2])) );
    
    InputSetpointLabel.setValue(trim(s[3]));
    InputGyroLabel.setValue(trim(s[4])); 
    InputAngleLabel.setValue(trim(s[5]));
    
    AngleOutLabel.setValue(trim(s[6])); 
    GyroOutLabel.setValue(trim(s[7]));
    
    kp = float(trim(s[8]));
    ki = float(trim(s[9]));
    kd = float(trim(s[10]));
    krp = float(trim(s[11]));
    kri = float(trim(s[12]));
    krd = float(trim(s[13]));

    PLabel.setValue(Float.toString(kp));
    ILabel.setValue(Float.toString(ki)); 
    DLabel.setValue(Float.toString(kd)); 
    
    PrLabel.setValue(Float.toString(krp));  
    IrLabel.setValue(Float.toString(kri));
    DrLabel.setValue(Float.toString(krd)); 
        
    //AMCurrent.setValue(trim(s[14]));
    DRCurrent.setValue(trim(s[15]));
    DRrCurrent.setValue(trim(s[16]));
    
    va = float(trim(s[17]));
    vb = float(trim(s[18]));
    vc = float(trim(s[19]));
    vd = float(trim(s[20]));
    
    alpha = float(trim(s[21]));
    AlphaLable.setValue(trim(s[21]));
    
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
      //SPField.setText(trim(s[2]));
      //InField.setText(trim(s[2]));
      //OutField.setText(trim(s[3]));
      /*
      PField.setValue(Float.toString(kp));
      IField.setValue(Float.toString(ki)); 
      DField.setValue(Float.toString(kd)); 
      
      PrField.setValue(Float.toString(krp));  
      IrField.setValue(Float.toString(kri));
      DrField.setValue(Float.toString(krd));    
      */
      
      // AMLabel.setValue(trim(s[14]));

      //DRCurrent.setValue(trim(s[15]));
      //DRrCurrent.setValue(trim(s[16]));

      justSent=false;
    } 

    if(!madeContact) madeContact=true;
  }else {
      
      print(read);
      
    }
}