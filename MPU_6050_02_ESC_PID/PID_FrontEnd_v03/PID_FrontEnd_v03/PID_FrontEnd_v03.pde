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
 *
 * The ControlP5 library is required to run this sketch.
 * files and install instructions can be found at
 * http://www.sojamo.de/libraries/controlP5/
 * 
 ********************************************************/

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

float last_throttle_position = 0;

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 1350;      // set the size of the 
int windowHeight = 690;     // form

float InScaleMin = -75;       // set the Y-Axis Min
float InScaleMax = 75;    // and Max for both
float OutScaleMin = -125;      // the top and 
float OutScaleMax = 125;    // bottom trends


int windowSpan = 4000;//0;//00;    // number of mS into the past you want to display
int refreshRate = 10;      // how often you want the graph to be reDrawn;

float displayFactor = 1; //display Time as Milliseconds
//float displayFactor = 1000; //display Time as Seconds
//float displayFactor = 60000; //display Time as Minutes

String outputFileName = ""; // if you'd like to output data to 
// a file, specify the path here

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[][] InputData = new int[3][arrayLength];
int[][] OutputData = new int[2][arrayLength];

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
controlP5.Button AMButton, DRButton, DRrButton, PIDSelector;

RadioButton serial_data_mode, tuning_mode;

float Input_Thrust, Input_Setpoint, Input_gyro, Input_angle, Output_angle, Output_gyro ;
byte i_serial_data_mode, i_tuning_mode;
float va,vb,vc,vd;

controlP5.Textlabel AMLabel, AMCurrent; 
controlP5.Textlabel InputThrustLabel, InputSetpointLabel, InputGyroLabel, InputAngleLabel, AngleOutLabel, GyroOutLabel;
controlP5.Textlabel PLabel, ILabel, DLabel, DRLabel, DRCurrent, DRrLabel, DRrCurrent, PrLabel, IrLabel, DrLabel ;

controlP5.Textfield SPField, /*InField, OutField,*/ PField, IField, DField, PrField, IrField, DrField;

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
  
  SPField= controlP5.addTextfield("Setpoint",10,100,60,20);           //   Buttons, Labels, and
 
  PField = controlP5.addTextfield("Kp",10,275,60,20); 
  IField = controlP5.addTextfield("Ki",10,325,60,20);
  DField = controlP5.addTextfield("Kd",10,375,60,20); 
  
  PrField = controlP5.addTextfield("Krp",110,275,60,20); 
  IrField = controlP5.addTextfield("Kri",110,325,60,20);
  DrField = controlP5.addTextfield("Krd",110,375,60,20);
  
  AMButton = controlP5.addButton("Toggle_AM",0.0,10,50,60,20);
  AMLabel = controlP5.addTextlabel("AM","Manual",12,72); 
  AMCurrent = controlP5.addTextlabel("AMCurrent","Manual",80,55); 
  
  group_x = 80; group_y=140;
  controlP5.addTextlabel("t1","Throttle",group_x-80,group_y);
  InputThrustLabel=controlP5.addTextlabel("InputThrustLabel","", group_x, group_y);
  controlP5.addTextlabel("t2","Setpoint",group_x-80,group_y+=15);
  InputSetpointLabel=controlP5.addTextlabel("InputSetpointLabel","",group_x,group_y);
  controlP5.addTextlabel("t3","Input Gyro",group_x-80,group_y+=15);
  InputGyroLabel=controlP5.addTextlabel("InputGyroLabel","",group_x,group_y);
  controlP5.addTextlabel("t4","Input Angle",group_x-80,group_y+=15);
  InputAngleLabel=controlP5.addTextlabel("InputAngleLabel","",group_x,group_y); 
  controlP5.addTextlabel("t5","Out Angle",group_x-80,group_y+=15);
  AngleOutLabel=controlP5.addTextlabel("AngleOutLabel","",group_x,group_y); 
  controlP5.addTextlabel("t6","Out Gyro",group_x-80,group_y+=15);
  GyroOutLabel=controlP5.addTextlabel("GyroOutLabel","",group_x,group_y);
  
  PLabel=controlP5.addTextlabel("Ps","",75,278);                    //
  ILabel=controlP5.addTextlabel("Is","",75,328);                    //
  DLabel=controlP5.addTextlabel("Ds","",75,378);                    //
  
  PrLabel=controlP5.addTextlabel("Pr","",175,278);                    //
  IrLabel=controlP5.addTextlabel("Ir","",175,328);                    //
  DrLabel=controlP5.addTextlabel("Dr","",175,378);                    //  
  
  DRButton = controlP5.addButton("Toggle_DR",0.0,10,425,60,20);      //
  DRLabel = controlP5.addTextlabel("DR","Dir",12,447);            //
  DRCurrent = controlP5.addTextlabel("DRCurrent","Dir",75,430);   //
  
  DRrButton = controlP5.addButton("Toggle_DRR",0.0,110,425,60,20);      //
  DRrLabel = controlP5.addTextlabel("DRr","Dir",120,447);            //
  DRrCurrent = controlP5.addTextlabel("DRrCurrent","Dir",175,430);   //  
   
  serial_data_mode = controlP5.addRadioButton("serial_data_mode_event")
         .setPosition(5,25)
         .setSize(10,10)
         .setColorForeground(color(120))
         .setColorActive(color(255))
         .setColorLabel(color(255))
         .setItemsPerRow(3)
         .setSpacingColumn(15)
         .addItem("AC",0)
         .addItem("BD",1)
         .addItem("YAW",2)
         ;
         
  serial_data_mode.activate(0);          
         
  tuning_mode = controlP5.addRadioButton("tuning_mode_event")
         .setPosition(5,5)
         .setSize(10,10)
         .setColorForeground(color(120))
         .setColorActive(color(255))
         .setColorLabel(color(255))
         .setItemsPerRow(4)
         .setSpacingColumn(33)
         .addItem("Flight",0)
         .addItem("Stable",1)
         .addItem("Rate",2)
         .addItem("Yaw",3)
         ;   
         
  tuning_mode.activate(0);         
         
   group_x = 10; group_y=460;         
   throttle_slider = controlP5.addSlider("throttle")
     .setPosition(group_x,group_y)
     .setSize(20,100)
     .setRange(0,800)
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
   
  controlP5.addButton("Send_To_Arduino",0.0,10,590,120,20);    

  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}

void draw()
{
  background(200);
  drawGraph();
  drawButtonArea();
}

void drawGraph()
{
  //draw Base, gridlines
  stroke(0);
  fill(230);
  rect(ioLeft, inputTop,ioWidth-1 , inputHeight);
  rect(ioLeft, outputTop, ioWidth-1, outputHeight);
  stroke(210);

  //Section Titles
  textFont(TitleFont);
  fill(255);
  text("PID Input / Setpoint",(int)ioLeft+10,(int)inputTop-5);
  text("PID Output",(int)ioLeft+10,(int)outputTop-5);


//  //GridLines and Titles
  textFont(AxisFont);
  
  //horizontal grid lines
  int interval = (int)inputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,inputTop+i*interval,ioRight-2,inputTop+i*interval);
    text(str((InScaleMax-InScaleMin)/5*(float)(5-i)+InScaleMin),ioRight+5,inputTop+i*interval+4);

  }
 interval = (int)outputHeight/5;
 for(int i=0;i<6;i++)
 {
   if(i>0&&i<5) line(ioLeft+1,outputTop+i*interval,ioRight-2,outputTop+i*interval);
   text(str((OutScaleMax-OutScaleMin)/5*(float)(5-i)+OutScaleMin),ioRight+5,outputTop+i*interval+4);
 }


//  //vertical grid lines and TimeStamps
//  int elapsedTime = millis();
//  interval = (int)ioWidth/vertCount;
//  int shift = elapsedTime*(int)ioWidth / windowSpan;
//  shift %=interval;

//  int iTimeInterval = windowSpan/vertCount;
//  float firstDisplay = (float)(iTimeInterval*(elapsedTime/iTimeInterval))/displayFactor;
//  float timeInterval = (float)(iTimeInterval)/displayFactor;
//  for(int i=0;i<vertCount;i++)
//  {
//    int x = (int)ioRight-shift-2-i*interval;

//    line(x,inputTop+1,x,inputTop+inputHeight-1);
//    line(x,outputTop+1,x,outputTop+outputHeight-1);    

//    float t = firstDisplay-(float)i*timeInterval;
//    if(t>=0)  text(str(t),x,outputTop+outputHeight+10);
//  }


  // add the latest data to the data Arrays.  the values need
  // to be massaged to get them to graph correctly.  they 
  // need to be scaled to fit where they're going, and 
  // because 0, 0 is the top left, we need to flip the values.
  // this is easier than having the user stand on their head
  // to read the graph.
  if(millis() > nextRefresh && madeContact)
  {
    nextRefresh += refreshRate;

    for(int i=nPoints-1;i>0;i--)
    {
      InputData[0][i]=InputData[0][i-1] ;
      InputData[1][i]=InputData[1][i-1] ;
      InputData[2][i]=InputData[2][i-1] ;
      //SetpointData[i]=SetpointData[i-1] ;
      OutputData[0][i]=OutputData[0][i-1] ;
      OutputData[1][i]=OutputData[1][i-1] ;
    }
    if (nPoints < arrayLength) nPoints++;

    //InputData[0][0] = int(inputHeight)-int(inputHeight*(Input_Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    InputData[0][0] = int(inputHeight)-int(inputHeight*(Output_gyro-InScaleMin)/(InScaleMax-InScaleMin));
    
    InputData[1][0] = int(inputHeight)-int(inputHeight*(Input_gyro-InScaleMin)/(InScaleMax-InScaleMin));
    InputData[2][0] = int(inputHeight)-int(inputHeight*(Input_angle-InScaleMin)/(InScaleMax-InScaleMin));    
    //SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0][0] = int(outputHeight)-int(outputHeight*(Output_angle-OutScaleMin)/(OutScaleMax-OutScaleMin));
    OutputData[1][0] = int(outputHeight)-int(outputHeight*(Output_gyro-OutScaleMin)/(OutScaleMax-OutScaleMin));
  }
  //draw lines for the input, setpoint, and output
  strokeWeight(2);

  for(int i=0; i<nPoints-2; i++)
  {
      int X1 = int(ioRight-2-float(i)*pointWidth);
      int X2 = int(ioRight-2-float(i+1)*pointWidth);
      boolean y1Above, y1Below, y2Above, y2Below;
      boolean drawLine=false;    
      int Y1, Y2;
  
    for( int iii = 0; iii<3; iii ++ ) {  
      //DRAW THE INPUT
      drawLine=true;
      if( iii == 0 )stroke(255, 0, 0 );
      else if (iii == 1 ) stroke(0,255,255);
      else stroke(255,0,255);
      
      Y1 = InputData[iii][i];
      Y2 = InputData[iii][i+1];
  
      y1Above = (Y1>inputHeight);                     // if both points are outside 
      y1Below = (Y1<0);                               // the min or max, don't draw the 
      y2Above = (Y2>inputHeight);                     // line.  if only one point is 
      y2Below = (Y2<0);                               // outside constrain it to the limit, 
      if(y1Above)                                     // and leave the other one untouched.
      {                                               //
        if(y2Above) drawLine=false;                   //
        else if(y2Below) {                            //
          Y1 = (int)inputHeight;                      //
          Y2 = 0;                                     //
        }                                             //
        else Y1 = (int)inputHeight;                   //
      }                                               //
      else if(y1Below)                                //
      {                                               //
        if(y2Below) drawLine=false;                   //
        else if(y2Above) {                            //
          Y1 = 0;                                     //
          Y2 = (int)inputHeight;                      //
        }                                             //
        else Y1 = 0;                                  //
      }                                               //
      else                                            //
      {                                               //
        if(y2Below) Y2 = 0;                           //
        else if(y2Above) Y2 = (int)inputHeight;       //
      }                                               //
  
      if(drawLine)
      {
        line(X1,Y1+inputTop, X2, Y2+inputTop);
      }
    }

/*
    //DRAW THE SETPOINT
    drawLine=true;
    stroke(0,255,0);
    Y1 = SetpointData[i];
    Y2 = SetpointData[i+1];

    y1Above = (Y1>(int)inputHeight);                // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>(int)inputHeight);                // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)(inputHeight);                    //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)(inputHeight);                 //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)(inputHeight);                    //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)(inputHeight);     //
    }                                               //

    if(drawLine)
    {
      line(X1, Y1+inputTop, X2, Y2+inputTop);
    }
*/
    //DRAW THE OUTPUT
    for( int iii = 0; iii<2; iii ++ ) {
      drawLine=true;
      stroke(255*iii,0,255);
      Y1 = OutputData[iii][i];
      Y2 = OutputData[iii][i+1];
  
      y1Above = (Y1>outputHeight);                   // if both points are outside 
      y1Below = (Y1<0);                              // the min or max, don't draw the 
      y2Above = (Y2>outputHeight);                   // line.  if only one point is 
      y2Below = (Y2<0);                              // outside constrain it to the limit, 
      if(y1Above)                                    // and leave the other one untouched.
      {                                              //
        if(y2Above) drawLine=false;                  //
        else if(y2Below) {                           //
          Y1 = (int)outputHeight;                    //
          Y2 = 0;                                    //
        }                                            //
        else Y1 = (int)outputHeight;                 //
      }                                              //
      else if(y1Below)                               //
      {                                              //
        if(y2Below) drawLine=false;                  //
        else if(y2Above) {                           //
          Y1 = 0;                                    //
          Y2 = (int)outputHeight;                    //
        }                                            //  
        else Y1 = 0;                                 //
      }                                              //
      else                                           //
      {                                              //
        if(y2Below) Y2 = 0;                          //
        else if(y2Above) Y2 = (int)outputHeight;     //
      }                                              //
  
      if(drawLine)
      {
        line(X1, outputTop + Y1, X2, outputTop + Y2);
      }
    }
  }
  strokeWeight(1);
}

void drawButtonArea()
{
  stroke(0);
  fill(100);
  rect(0, 0, ioLeft, windowHeight);
}

void controlEvent(ControlEvent theEvent) {
  
  if(theEvent.isFrom(tuning_mode)) {
    if( tuning_mode.getState(0) ) i_tuning_mode = (byte) 0;
    else if( tuning_mode.getState(1) ) i_tuning_mode = (byte) 1;
    else if( tuning_mode.getState(2) ) i_tuning_mode = (byte) 2;
    else if( tuning_mode.getState(3) ) i_tuning_mode = (byte) 3;
    //else i_tuning_mode = 0;   
    
    if( tuning_mode.getState(0) ) Send_To_Arduino2(101.0, 0);
    else if( tuning_mode.getState(1) ) Send_To_Arduino2(101.0, 1);
    else if( tuning_mode.getState(2) ) Send_To_Arduino2(101.0, 2);
    else if( tuning_mode.getState(3) ) Send_To_Arduino2(101.0, 3);    
    
    //Send_To_Arduino();
    //Send_To_Arduino2(101.0, (float)i_tuning_mode);
    
    return;
  } 
  
  if(theEvent.isFrom(serial_data_mode)) {
    if( serial_data_mode.getState(0) ) i_serial_data_mode = (byte) 0;
    else if( serial_data_mode.getState(1) ) i_serial_data_mode = (byte) 1;
    else if( serial_data_mode.getState(2) ) i_serial_data_mode = (byte) 2;
    //else i_serial_data_mode = 0;
    
    if( serial_data_mode.getState(0) ) Send_To_Arduino2(100.0, 0.0);
    else if( serial_data_mode.getState(1) ) Send_To_Arduino2(100.0, 1.0);
    else if( serial_data_mode.getState(2) ) Send_To_Arduino2(100.0, 2.0);    
    
    //Send_To_Arduino2(100.0, (float)i_serial_data_mode);
    
    return;
  }   
}

float kp=0,ki=0,kd=0,krp=0,kri=0,krd=0,yawKp=0,yawKi=0,yawKd=0;

float cur_throttle ;
void keyPressed() {
    switch(key) {
      case ' ':
        cur_throttle = 0.0;
        Send_To_Arduino2(0.0, cur_throttle);         
        break;
      case 'w':
        outputFileName = "c:\\temp\\drone_" + year()+month()+day()+hour()+minute()+second()+".txt";
        output = createWriter(outputFileName);
        output.println("t S selected.pot.tuning_serial.data.mode INPUT_THRUST SETPOINT inputgyro inputypr outputypr outputrate kp ki kd rKp rKi rKd A/M D/R rD/R va vb vc vd E");        
        break;
      case 'e':
        output.flush(); // Writes the remaining data to the file
        output.close(); // Finishes the file
        outputFileName = "";        
        break;
        
      case '0':
        if( yawKp < 10.0 ) yawKp += 0.1;
        Send_To_Arduino2(7.0, yawKp);
        break;
      case 'p':
        if( yawKp > 0.0 ) yawKp -= 0.1;
        Send_To_Arduino2(7.0, yawKp);      
        break;

      case '-':
        if( yawKi < 10.0 ) yawKi += 0.1;
        Send_To_Arduino2(8.0, yawKi);
        break;
      case '[':
        if( yawKi > 0.0 ) yawKi -= 0.1;
        Send_To_Arduino2(8.0, yawKi);      
        break;      

      case '-':
        if( yawKd < 10.0 ) yawKd += 0.1;
        Send_To_Arduino2(9.0, yawKd);
        break;
      case '[':
        if( yawKd > 0.0 ) yawKd -= 0.1;
        Send_To_Arduino2(9.0, yawKd);      
        break;      
      
      
      case 'a':
        if(cur_throttle < 900.0) cur_throttle += 10.0;
        Send_To_Arduino2(0.0, cur_throttle);        
        break;
      case 'z':
        if(cur_throttle > 0.0) cur_throttle -= 20.0;
        Send_To_Arduino2(0.0, cur_throttle);           
        break;
      case '7':
        if( kp < 10.0) kp += 0.01;
        Send_To_Arduino2(1.0, kp);        
        break;
      case '8':
        if( ki < 10.0 ) ki += 0.01; 
        Send_To_Arduino2(2.0, ki);        
        break;
      case '9':  
        if( kd < 10.0 ) kd += 0.01; 
        Send_To_Arduino2(3.0, kd);        
        break;
      case 'u':
        if( kp > 0.0 ) kp -= 0.01; 
        Send_To_Arduino2(1.0, kp);        
        break;
      case 'i':
        if( ki > 0.0 ) ki -= 0.01;
        Send_To_Arduino2(2.0, ki);        
        break;
      case 'o':
        if( kd > 0.0 ) kd -= 0.01;
        Send_To_Arduino2(3.0, kd);        
        break;
        
      case 'j':
        if( krp < 10.0 ) krp += 0.01;
        Send_To_Arduino2(4.0, krp);        
        break;
      case 'k':
        if( kri < 10 ) kri += 0.01;
        Send_To_Arduino2(5.0, kri);        
        break;
      case 'l':
        if( krd < 10.0 ) krd += 0.01;
        Send_To_Arduino2(6.0, krd);        
        break;
      case 'm':
        if( krp > 0.0 ) krp  -= 0.01;
        Send_To_Arduino2(4.0, krp);        
        break;
      case ',':
        if( kri > 0.0 ) kri -= 0.01;
        Send_To_Arduino2(5.0, kri);        
        break;
      case '.':
        if( krd > 0.0 ) krd -= 0.01;
        Send_To_Arduino2(6.0, krd);        
        break;
        
    }
    last_throttle_position = cur_throttle;
   
    // Send_To_Arduino();
}

void Toggle_AM() {
  
  if(AMLabel.get().getText()=="Manual") 
  {
    AMLabel.setValue("Automatic");
  }
  else
  {
    AMLabel.setValue("Manual");   
  }
}


void Toggle_DR() {
  if(DRLabel.get().getText()=="Dir") 
  {
    DRLabel.setValue("Rev");
  }
  else
  {
    DRLabel.setValue("Dir");   
  }
}

void Toggle_DRR() {
  if(DRrLabel.get().getText()=="Dir") 
  {
    DRrLabel.setValue("Rev");
  }
  else
  {
    DRrLabel.setValue("Dir");   
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
  
  //println( FloatArray2ByteArray(toSend) );
  //println( "----------" );
  //println( floatArrayToByteArray(toSend) );
  
  justSent=true;  
}

void Send_To_Arduino2x(float command, float value)
{
  float[] toSend = new float[2]; 
  toSend[0] = command;
  toSend[1] = value;
  
  byte[] bbb = new byte[toSend.length * 4];
  byte [] dddd = floatArrayToByteArray(toSend);
  for(int i=0; i< dddd.length; i++ )
  {
    bbb[i] = dddd[i];
  }
  if( myPort != null ) {
    myPort.write(bbb);
  }  
  
  println( bbb );
  justSent=true;
}

// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Arduinox()
{
  float[] toSend = new float[11];

  toSend[0] = float(SPField.getText());
  toSend[1] = 0.0; //float(InField.getText());
  toSend[2] = i_serial_data_mode; //float(OutField.getText());
  
  toSend[3] = kp;
  toSend[4] = ki;
  toSend[5] = kd;
  
  toSend[6] = krp;
  toSend[7] = kri;
  toSend[8] = krd;
  
  toSend[9] = last_throttle_position;
  toSend[10] = last_throttle_position;
  
  
  byte a = (AMLabel.get().getText()=="Manual")?(byte)0:(byte)1;
  byte d = (DRLabel.get().getText()=="Dir")?(byte)0:(byte)1;
  byte dr = (DRrLabel.get().getText()=="Dir")?(byte)0:(byte)1;
  
  //println(i_tuning_mode);
  //println(i_serial_data_mode);
  
  byte pid_tuning = i_tuning_mode ; 
  
  byte serial_send_mode = i_serial_data_mode;
  
  byte[] bbb = new byte[toSend.length * 4 + 5];
  
  bbb[0] = a;
  bbb[1] = d;
  bbb[2] = dr;
  bbb[3] = pid_tuning;
  bbb[4] = serial_send_mode;
  
  byte [] dddd = floatArrayToByteArray(toSend);
  for(int i=0; i< dddd.length; i++ )
  {
    bbb[i+5] = dddd[i];
  }
  if( myPort != null ) {
    myPort.write(bbb);
  }
  
  println( bbb );
  
  justSent=true;
} 

byte[] FloatArray2ByteArray(float[] values)
{
  ByteBuffer buffer = ByteBuffer.allocate(4 * values.length);
  
  for (float value : values){
      buffer.putFloat(value);
  }
  return buffer.array();
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


//take the string the arduino sends us and parse it
void serialEvent(Serial myPort)
{
// S thrust _ setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder _ dir/inder E
// S 0_0 0.00 0.00 0.00 0.15 0.00 0.00 3.000 0.000 0.000 0.960 0.000 0.096 Manual Dir Dir 1100 1100 1100 1100 E

// HEADER
// t S selected.pot.tuning_serial.data.mode INPUT_THRUST SETPOINT inputgyro inputypr outputypr outputrate kp ki kd rKp rKi rKd A/M D/R rD/R va vb vc vd E
  
  String read = myPort.readStringUntil(10);
  
  // print(read);
  
  if(outputFileName!="") output.print(str(millis())+ " " + read);
  String[] s = split(read, " ");

  if (s.length == 22)
  {
    if(! trim(s[0]).equals("S") ) return;
    if(! trim(s[21]).equals("E") ) return;
    
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
    
    AMCurrent.setValue(trim(s[14]));
    DRCurrent.setValue(trim(s[15]));
    DRrCurrent.setValue(trim(s[16]));
    
    va = float(trim(s[17]));
    vb = float(trim(s[18]));
    vc = float(trim(s[19]));
    vd = float(trim(s[20]));
    
    controlP5.getController("va").setValue(va);
    controlP5.getController("vb").setValue(vb);
    controlP5.getController("vc").setValue(vc);
    controlP5.getController("vd").setValue(vd);
    
    if( va > vc )
    {
      int c = (int)abs(va-vc)*20;
      controlP5.getController("va").setColorForeground(color(c, 0, 255));
      controlP5.getController("vc").setColorForeground(color(0, 0, 255));    
    }
    else
    {
      int c = (int)abs(va-vc)*20;
      controlP5.getController("va").setColorForeground(color(0, 0, 255));
      controlP5.getController("vc").setColorForeground(color(c, 0, 255));  
    }
    
    if( vb > vd )
    {
      int c = (int)abs(vb-vd)*20;      
      controlP5.getController("vb").setColorForeground(color(c, 0, 255));
      controlP5.getController("vd").setColorForeground(color(0, 0, 255));    
    }
    else
    {
      int c = (int)abs(vb-vd)*20;
      controlP5.getController("vb").setColorForeground(color(0, 0, 255));
      controlP5.getController("vd").setColorForeground(color(c, 0, 255));  
    }    

    if(justSent)                     
    {                                
      //SPField.setText(trim(s[2]));
      //InField.setText(trim(s[2]));
      //OutField.setText(trim(s[3]));
      
      PField.setValue(Float.toString(kp));
      IField.setValue(Float.toString(ki)); 
      DField.setValue(Float.toString(kd)); 
      
      PrField.setValue(Float.toString(krp));  
      IrField.setValue(Float.toString(kri));
      DrField.setValue(Float.toString(krd));           
      
      AMLabel.setValue(trim(s[14]));

      DRCurrent.setValue(trim(s[15]));
      DRrCurrent.setValue(trim(s[16]));

      justSent=false;
    } 

    if(!madeContact) madeContact=true;
  }else {
      
      print(read);
      
    }
}