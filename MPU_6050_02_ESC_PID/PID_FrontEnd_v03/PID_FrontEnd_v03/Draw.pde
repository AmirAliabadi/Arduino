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
      
      OutputData[0][i]=OutputData[0][i-1] ;
      OutputData[1][i]=OutputData[1][i-1] ;

      OutputData[2][i]=OutputData[2][i-1] ;
      OutputData[3][i]=OutputData[3][i-1] ;
      OutputData[4][i]=OutputData[4][i-1] ;      

      
    }
    if (nPoints < arrayLength) nPoints++;

    //InputData[0][0] = int(inputHeight)-int(inputHeight*(Input_Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    InputData[0][0] = int(inputHeight)-int(inputHeight*(Output_gyro-InScaleMin)/(InScaleMax-InScaleMin));
    InputData[1][0] = int(inputHeight)-int(inputHeight*(Input_gyro-InScaleMin)/(InScaleMax-InScaleMin));
    InputData[2][0] = int(inputHeight)-int(inputHeight*(Input_angle-InScaleMin)/(InScaleMax-InScaleMin));    

    OutputData[0][0] = int(outputHeight)-int(outputHeight*(Output_angle-OutScaleMin)/(OutScaleMax-OutScaleMin));
    OutputData[1][0] = int(outputHeight)-int(outputHeight*(Output_gyro-OutScaleMin)/(OutScaleMax-OutScaleMin));
    
    OutputData[2][0] = int(outputHeight)-int(outputHeight*(pterm_rate-OutScaleMin)/(OutScaleMax-OutScaleMin));
    OutputData[3][0] = int(outputHeight)-int(outputHeight*(iterm_rate-OutScaleMin)/(OutScaleMax-OutScaleMin));
    OutputData[4][0] = int(outputHeight)-int(outputHeight*(dterm_rate-OutScaleMin)/(OutScaleMax-OutScaleMin));
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
    for( int iii = 0; iii<5; iii ++ ) {
      drawLine=true;
      stroke(iii*42,0,255);
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