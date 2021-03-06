/********************************************************
 * Arduino PID Tuning Front-End,  Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 *
 * This application is designed to interface with an
 * arduino running the PID Library.  From this Control
 * Panel you can observe & adjust PID performance in 
 * real time
 *a
 * The ControlP5 library is required to run this sketch.
 * files and install instructions can be found at
 * http://www.sojamo.de/libraries/controlP5/
 * 
 ********************************************************/

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

float last_throttle_position = 0;

float kp=0,ki=0,kd=0,krp=0,kri=0,krd=0;
float cur_throttle ;
float alpha =0.0 ;
float pid_refresh_rate=10.0 ;

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 1350;      // set the size of the 
int windowHeight = 690;     // form

float InScaleMin = -75;       // set the Y-Axis Min
float InScaleMax = 75;    // and Max for both
float OutScaleMin = -125;      // the top and 
float OutScaleMax = 125;    // bottom trends


int windowSpan = 2000;//0;//00;    // number of mS into the past you want to display
int refreshRate = 10;      // how often you want the graph to be reDrawn;

float displayFactor = 1; //display Time as Milliseconds
//float displayFactor = 1000; //display Time as Seconds
//float displayFactor = 60000; //display Time as Minutes

// if you'd like to output data to 
// a file, specify the path here
String outputFileName = ""; 

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[][] InputData = new int[3][arrayLength];
int[][] OutputData = new int[5][arrayLength];

float inputTop = 25;
float inputHeight = (windowHeight-70)*2/3;
float outputTop = inputHeight+50;
float outputHeight = (windowHeight-70)*1/3;

float ioLeft = 210, ioWidth = windowWidth-ioLeft-50;
float ioRight = ioLeft+ioWidth;
float pointWidth= (ioWidth)/float(arrayLength-1);

int vertCount = 10;
int nPoints = 0;

boolean madeContact =false;
boolean justSent = true;

Serial myPort;

ControlP5 controlP5;
controlP5.Button DRButton, DRrButton;

RadioButton serial_data_mode; //, tuning_mode;

float Input_Thrust, Input_Setpoint, Input_gyro, Input_angle, Output_angle, Output_gyro ;
byte i_serial_data_mode; 
float va,vb,vc,vd;
float pterm_stable, iterm_stable, dterm_stable;
float pterm_rate, iterm_rate, dterm_rate;

controlP5.Textlabel InputThrustLabel, InputSetpointLabel, InputGyroLabel, InputAngleLabel, AngleOutLabel, GyroOutLabel, AlphaLable, PidRefreshIntervalLabel;
controlP5.Textlabel PTermRateLabel, ITermRateLabel, DTermRateLabel;
controlP5.Textlabel PLabel, ILabel, DLabel, DRLabel, DRCurrent, DRrLabel, DRrCurrent, PrLabel, IrLabel, DrLabel ;

controlP5.Textlabel SPField; 

controlP5.Slider throttle_slider;

PrintWriter output;
PFont AxisFont, TitleFont; 

void setup()
{
  int group_x =0;
  int group_y =0;
  
  frameRate(30); // 30
  size(1350 , 690);

  println(Serial.list());   // * Initialize Serial
  
  if( Serial.list().length > 0 ) {
    myPort = new Serial(this, Serial.list()[0], 115200);                //   Communication with
    myPort.bufferUntil(10);         //   the Arduino
  }
  
  controlP5 = new ControlP5(this);                                    // * Initialize the various
  
  group_x = 80; group_y=140;
  
  controlP5.addTextlabel("t-1","Setpoint",group_x-80,group_y);  
  SPField= controlP5.addTextlabel("Setpoint","", group_x, group_y);
  
  controlP5.addTextlabel("t1","Throttle",group_x-80,group_y+=15);
  InputThrustLabel=controlP5.addTextlabel("InputThrustLabel","", group_x, group_y);
  
  controlP5.addTextlabel("t2","Setpoint",group_x-80,group_y+=15);
  InputSetpointLabel=controlP5.addTextlabel("InputSetpointLabel","",group_x,group_y);

  controlP5.addTextlabel("t4","Input Angle",group_x-80,group_y+=15);
  InputAngleLabel=controlP5.addTextlabel("InputAngleLabel","",group_x,group_y); 
  controlP5.addTextlabel("t3","Input Gyro",group_x-80,group_y+=15);
  InputGyroLabel=controlP5.addTextlabel("InputGyroLabel","",group_x,group_y);

  controlP5.addTextlabel("t5","Out Angle",group_x-80,group_y+=15);
  AngleOutLabel=controlP5.addTextlabel("AngleOutLabel","",group_x,group_y); 
  controlP5.addTextlabel("t6","Out Gyro",group_x-80,group_y+=15);
  GyroOutLabel=controlP5.addTextlabel("GyroOutLabel","",group_x,group_y);

  controlP5.addTextlabel("t7","Alpha",group_x-80,group_y+=15);
  AlphaLable=controlP5.addTextlabel("AlphaLable","",group_x,group_y);

  controlP5.addTextlabel("t71","PID Refresh",group_x-80,group_y+=15);
  PidRefreshIntervalLabel=controlP5.addTextlabel("PidRefreshIntervalLabel","",group_x,group_y);
  
  controlP5.addTextlabel("t72","PTerm Rate",group_x-80,group_y+=15);
  PTermRateLabel=controlP5.addTextlabel("PTermRateLabel","10",group_x,group_y);
  controlP5.addTextlabel("t73","ITerm Rate",group_x-80,group_y+=15);
  ITermRateLabel=controlP5.addTextlabel("ITermRateLabel","10",group_x,group_y);
  controlP5.addTextlabel("t74","DTerm Rate",group_x-80,group_y+=15);
  DTermRateLabel=controlP5.addTextlabel("DTermRateLabel","10",group_x,group_y);
  
  controlP5.addTextlabel("t8","Stable",group_x-80,group_y+=35);  
  controlP5.addTextlabel("t81","Rate",group_x,group_y);    
  
  controlP5.addTextlabel("t9","P: ",group_x-80,group_y+=15);    
  PLabel=controlP5.addTextlabel("Ps","0.00",group_x-70,group_y);                    //
  PrLabel=controlP5.addTextlabel("Pr","0.00",group_x,group_y);                    //

  controlP5.addTextlabel("t10","I: ",group_x-80,group_y+=15);    
  ILabel=controlP5.addTextlabel("Is","0.00",group_x-70,group_y);                    //
  IrLabel=controlP5.addTextlabel("Ir","0.00",group_x,group_y);                    //
  
  controlP5.addTextlabel("t11","D: ",group_x-80,group_y+=15);    
  DLabel=controlP5.addTextlabel("Ds","0.00",group_x-70,group_y);                    //
  DrLabel=controlP5.addTextlabel("Dr","0.00",group_x,group_y);                    //  
  
  DRButton = controlP5.addButton("Toggle_DR",0.0,10,425,60,20);      //
  DRLabel = controlP5.addTextlabel("DR","Dir",12,447);            //
  DRCurrent = controlP5.addTextlabel("DRCurrent","Dir",75,430);   //
  
  DRrButton = controlP5.addButton("Toggle_DRR",0.0,110,425,60,20);      //
  DRrLabel = controlP5.addTextlabel("DRr","Dir",120,447);            //
  DRrCurrent = controlP5.addTextlabel("DRrCurrent","Dir",175,430);   //  
   
  serial_data_mode = controlP5.addRadioButton("serial_data_mode_event")
         .setPosition(5,25)
         .setSize(20,20)
         .setColorForeground(color(120))
         .setColorActive(color(255))
         .setColorLabel(color(255))
         .setItemsPerRow(3)
         .setSpacingColumn(25)
         .addItem("AC",0)
         .addItem("BD",1)
         .addItem("YAW",2)
         ;
         
  serial_data_mode.activate(0);          
    
   group_x = 10; group_y=460;         
   throttle_slider = controlP5.addSlider("throttle")
     .setPosition(group_x,group_y)
     .setSize(20,100)
     .setRange(0,1000)
     .setValue(0)
     ;         
         
   controlP5.addSlider("va")
     .setPosition(group_x+=40,group_y)
     .setSize(20,100)
     .setRange(1000,2000)
     .setValue(0)
     ;

  controlP5.addSlider("vb")
    .setPosition(group_x+=40,group_y)
     .setSize(20,100)
     .setRange(1000,2000)
     .setValue(0)
     ;
     
  controlP5.addSlider("vc")
    .setPosition(group_x+=40,group_y)
     .setSize(20,100)
     .setRange(1000,2000)
     .setValue(0)
     ;
     
  controlP5.addSlider("vd")
     .setPosition(group_x+=40,group_y)
     .setSize(20,100)
     .setRange(1000,2000)
     .setValue(0)
     ;           
     
   // reposition the Label for controller 'slider'
  controlP5.getController("throttle").getValueLabel().align(ControlP5.TOP, ControlP5.TOP).setPaddingX(10);
  controlP5.getController("va").getValueLabel().align(ControlP5.TOP, ControlP5.TOP).setPaddingX(10);
  controlP5.getController("vb").getValueLabel().align(ControlP5.TOP, ControlP5.TOP).setPaddingX(10);
  controlP5.getController("vc").getValueLabel().align(ControlP5.TOP, ControlP5.TOP).setPaddingX(10);
  controlP5.getController("vd").getValueLabel().align(ControlP5.TOP, ControlP5.TOP).setPaddingX(10);
   
  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}


void controlEvent(ControlEvent theEvent) {
    
  if(theEvent.isFrom(serial_data_mode)) {
    if( serial_data_mode.getState(0) ) i_serial_data_mode = (byte) 0;
    else if( serial_data_mode.getState(1) ) i_serial_data_mode = (byte) 1;
    else if( serial_data_mode.getState(2) ) i_serial_data_mode = (byte) 2;
    else i_serial_data_mode = 0;
    
    if( serial_data_mode.getState(0) ) Send_To_Arduino2(100.0, 0.0);
    else if( serial_data_mode.getState(1) ) Send_To_Arduino2(100.0, 1.0);
    else if( serial_data_mode.getState(2) ) Send_To_Arduino2(100.0, 2.0);    
        
    return;
  }   
}

void Toggle_DR() {
  if(DRCurrent.get().getText().equals("D")) 
  {
    Send_To_Arduino2( 101, 1.0);        
  }
  else
  {
    Send_To_Arduino2( 101, 0.0);
  }
}

void Toggle_DRR() {
  println(DRrCurrent.get().getText());    

  if(DRrCurrent.get().getText().equals("D")) 
  {
    Send_To_Arduino2( 102, 1.0);        
  }
  else
  {
    Send_To_Arduino2( 102, 0.0);        
  }
}

void Send_To_Arduino2(float command, float value)
{
  float[] toSend = new float[2]; 
  toSend[0] = command;
  toSend[1] = value;
  
  if( myPort != null ) {
    myPort.write(floatArrayToByteArray(toSend));
  }    
    
  justSent=true;  
}

byte[] floatArrayToByteArray(float[] input)
{
  int len = 4*input.length;
  byte[] b = new byte[4];
  byte[] out = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  for(int i=0;i<input.length;i++) 
  {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) out[j+i*4]=b[3-j];
  }
  return out;
}