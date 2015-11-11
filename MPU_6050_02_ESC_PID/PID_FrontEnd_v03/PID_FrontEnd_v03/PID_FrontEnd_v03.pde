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

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 1500;      // set the size of the 
int windowHeight = 768;     // form

float InScaleMin = -50;       // set the Y-Axis Min
float InScaleMax = 50;    // and Max for both
float OutScaleMin = -100;      // the top and 
float OutScaleMax = 100;    // bottom trends


int windowSpan = 3000;//00;    // number of mS into the past you want to display
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
int[] InputData = new int[arrayLength];     //we might not need them this big, but
int[] SetpointData = new int[arrayLength];  // this is worst case
int[][] OutputData = new int[2][arrayLength];
//int[] OutputData = new int[arrayLength];

float inputTop = 25;
float inputHeight = (windowHeight-70)*2/3;
float outputTop = inputHeight+50;
float outputHeight = (windowHeight-70)*1/3;

float ioLeft = 200, ioWidth = windowWidth-ioLeft-50;
float ioRight = ioLeft+ioWidth;
float pointWidth= (ioWidth)/float(arrayLength-1);

int vertCount = 10;

int nPoints = 0;

float Input_Setpoint, Input_gyro, Input_angle, Output_angle, Output_gyro ;

boolean madeContact =false;
boolean justSent = true;

Serial myPort;

ControlP5 controlP5;
controlP5.Button AMButton, DRButton, DRrButton, PIDSelector;

controlP5.Textlabel AMLabel, AMCurrent, InLabel, OutLabel, SPLabel, PLabel, ILabel, DLabel, DRLabel, DRCurrent, DRrLabel, DRrCurrent, PrLabel, IrLabel, DrLabel ;
controlP5.Textlabel InputSetpointLabel, InputGyroLabel, InputAngleLabel, AngleOutLabel, GyroOutLable ;

controlP5.Textlabel TogglePIDLable ;

controlP5.Textfield SPField, InField, OutField, PField, IField, DField, PrField, IrField, DrField;

PrintWriter output;
PFont AxisFont, TitleFont; 

void setup()
{
  frameRate(300); // 30
  size(1500 , 768);

  println(Serial.list());                                           // * Initialize Serial
  myPort = new Serial(this, Serial.list()[0], 115200);                //   Communication with
  myPort.bufferUntil(10);                                           //   the Arduino

  controlP5 = new ControlP5(this);                                  // * Initialize the various
  
  SPField= controlP5.addTextfield("Setpoint",10,100,60,20);         //   Buttons, Labels, and
  InField = controlP5.addTextfield("Input",10,150,60,20);           //   Text Fields we'll be
  OutField = controlP5.addTextfield("Output",10,200,60,20);         //   using
  
  PField = controlP5.addTextfield("Kp",10,275,60,20);          //
  IField = controlP5.addTextfield("Ki",10,325,60,20);          //
  DField = controlP5.addTextfield("Kd",10,375,60,20);          //
  
  PrField = controlP5.addTextfield("Krp",110,275,60,20);          //
  IrField = controlP5.addTextfield("Kri",110,325,60,20);          //
  DrField = controlP5.addTextfield("Krd",110,375,60,20);          //
  
  AMButton = controlP5.addButton("Toggle_AM",0.0,10,50,60,20);      //
  AMLabel = controlP5.addTextlabel("AM","Manual",12,72);            //
  AMCurrent = controlP5.addTextlabel("AMCurrent","Manual",80,65);   //
  controlP5.addButton("Send_To_Arduino",0.0,10,475,120,20);         //
  
  controlP5.addButton("Toggle_PID",0.0, 10, 500, 120,20);
  TogglePIDLable = controlP5.addTextlabel("TogglePIDLable","Stable",80,525);    
  
  SPLabel=controlP5.addTextlabel("SP","3",80,103);                  //
  InLabel=controlP5.addTextlabel("In","1",80,153);                  //
  OutLabel=controlP5.addTextlabel("Out","2",80,203);                //
  
  PLabel=controlP5.addTextlabel("Ps","4",75,278);                    //
  ILabel=controlP5.addTextlabel("Is","5",75,328);                    //
  DLabel=controlP5.addTextlabel("Ds","6",75,378);                    //
  
  PrLabel=controlP5.addTextlabel("Pr","4",175,278);                    //
  IrLabel=controlP5.addTextlabel("Ir","5",175,328);                    //
  DrLabel=controlP5.addTextlabel("Dr","6",175,378);                    //  
  
  DRButton = controlP5.addButton("Toggle_DR",0.0,10,425,60,20);      //
  DRLabel = controlP5.addTextlabel("DR","Dir",12,447);            //
  DRCurrent = controlP5.addTextlabel("DRCurrent","Dir",75,430);   //
  
  DRrButton = controlP5.addButton("Toggle_DRR",0.0,110,425,60,20);      //
  DRrLabel = controlP5.addTextlabel("DRr","Dir",120,447);            //
  DRrCurrent = controlP5.addTextlabel("DRrCurrent","Dir",175,430);   //  

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
      InputData[i]=InputData[i-1] ;
      SetpointData[i]=SetpointData[i-1] ;
      OutputData[0][i]=OutputData[0][i-1] ;
      OutputData[1][i]=OutputData[1][i-1] ;
    }
    if (nPoints < arrayLength) nPoints++;

    InputData[0] = int(inputHeight)-int(inputHeight*(Input-InScaleMin)/(InScaleMax-InScaleMin));
    SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0][0] = int(outputHeight)-int(outputHeight*(Output_1-OutScaleMin)/(OutScaleMax-OutScaleMin));
    OutputData[1][0] = int(outputHeight)-int(outputHeight*(Output_2-OutScaleMin)/(OutScaleMax-OutScaleMin));
  }
  //draw lines for the input, setpoint, and output
  strokeWeight(2);
  for(int i=0; i<nPoints-2; i++)
  {
    int X1 = int(ioRight-2-float(i)*pointWidth);
    int X2 = int(ioRight-2-float(i+1)*pointWidth);
    boolean y1Above, y1Below, y2Above, y2Below;


    //DRAW THE INPUT
    boolean drawLine=true;
    stroke(255,0,0);
    int Y1 = InputData[i];
    int Y2 = InputData[i+1];

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

void Toggle_PID() {
  if(TogglePIDLable.get().getText()=="Stable") 
  {
    TogglePIDLable.setValue("Rate");
  }
  else
  {
    TogglePIDLable.setValue("Stable");   
  }  
}

// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Arduino()
{
  float[] toSend = new float[9];

  toSend[0] = float(SPField.getText());
  toSend[1] = float(InField.getText());
  toSend[2] = float(OutField.getText());
  
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  
  toSend[6] = float(PrField.getText());
  toSend[7] = float(IrField.getText());
  toSend[8] = float(DrField.getText());  
  
  Byte a = (AMLabel.get().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.get().getText()=="Dir")?(byte)0:(byte)1;
  Byte dr = (DRrLabel.get().getText()=="Dir")?(byte)0:(byte)1;
  
  Byte pid_tuning = (TogglePIDLable.get().getText()=="Stable")?(byte)0:(byte)1;
  
  byte[] bbb = new byte[toSend.length * 4 + 4];
  
  bbb[0] = a;
  bbb[1] = d;
  bbb[2] = dr;
  bbb[3] = pid_tuning;
  
  byte [] dddd = floatArrayToByteArray(toSend);
  for(int i=0; i< dddd.length; i++ )
  {
    bbb[i+4] = dddd[i];
  }
  myPort.write(bbb);
  
  //myPort.write(a);
  //myPort.write(d);
  //myPort.write(floatArrayToByteArray(toSend));
  justSent=true;
} 


byte[] floatArrayToByteArray(float[] input)
{
  int len = 4*input.length;
  int index=0;
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
// PID setpoint _ input_gyro _ input_angle  _ output_angle _ output_gyro _ pid.p _ pid.i _ pid.d _ rat.p _ rat.i _ rat.d _ man/auto _ dir/inder _ dir/inder
  
  String read = myPort.readStringUntil(10);
  
  print(read);
  
  if(outputFileName!="") output.print(str(millis())+ " "+read);
  String[] s = split(read, " ");

  if (s.length == 15)
  {
    Input_Setpoint = float(s[1]);           // * pull the information
    Input_gyro = float(s[2]);              //   we need out of the
    Input_angle = float(s[3]);              //   we need out of the    
    
    Output_angle = float(s[4]);             //   string and put it
    Output_gyro = float(s[5]);             //   string and put it
    
    InputSetpointLabel.setValue(Input_Setpoint);
    InputGyroLabel.setValue(Input_gyro);           //
    InputAngleLabel.setValue(Input_gyro);           //
    
    AngleOutLabel.setValue(Output_angle);    //
    GyroOutLable.setValue(Output_gyro);
    
    PLabel.setValue(trim(s[6]));      //
    ILabel.setValue(trim(s[7]));      //
    DLabel.setValue(trim(s[8]));      //
    
    PrLabel.setValue(trim(s[9]));      //
    IrLabel.setValue(trim(s[10]));      //
    DrLabel.setValue(trim(s[11]));      //    
    
    AMCurrent.setValue(trim(s[12]));   //
    DRCurrent.setValue(trim(s[13]));
    DRrCurrent.setValue(trim(s[14]));
    
    if(justSent)                      // * if this is the first read
    {                                 //   since we sent values to 
      SPField.setText(trim(s[1]));    //   the arduino,  take the
      InField.setText(trim(s[2]));    //   current values and put
      OutField.setText(trim(s[3]));   //   them into the input fields
      
      PField.setText(trim(s[5]));
      IField.setText(trim(s[6]));
      DField.setText(trim(s[7]));
      
      PrField.setText(trim(s[8]));
      IrField.setText(trim(s[9]));
      DrField.setText(trim(s[10]));      
      
      AMLabel.setValue(trim(s[11]));

      DRCurrent.setValue(trim(s[12]));
      DRrCurrent.setValue(trim(s[12]));

      justSent=false;
    }

    if(!madeContact) madeContact=true;
  }
}