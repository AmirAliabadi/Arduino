// import libraries
import java.awt.Frame;
import java.awt.BorderLayout;
import controlP5.*; // http://www.sojamo.de/libraries/controlP5/
import processing.serial.*;

/* SETTINGS BEGIN */

// Serial port to connect to
String serialPortName = "/dev/tty.usbmodem1411";

// If you want to debug the plotter without using a real serial port set this to true
boolean mockupSerial = false;

/* SETTINGS END */

Serial serialPort; // Serial port object

// interface stuff
ControlP5 cp5;

String[] control_inputs = new String[5];
boolean motors_on = false;
int max_data_points = 7;

// Settings for the plotter are saved in this file
JSONObject plotterConfigJSON;

// plots
Graph BarChart = new Graph(225, 70, 600, 150, color(20, 20, 200));
Graph LineGraph = new Graph(225, 350, 600, 200, color (20, 20, 200));
float[] barChartValues = new float[max_data_points];
float[][] lineGraphValues = new float[max_data_points][100];
float[] lineGraphSampleNumbers = new float[100];
color[] graphColors = new color[12];

// helper for saving the executing path
String topSketchPath = "";

void setup() {
  surface.setTitle("Realtime plotter");
  size(890, 650);

  // set line graph colors
  graphColors[0] = color(131, 255, 20);
  graphColors[1] = color(232, 158, 12);
  graphColors[2] = color(255, 0, 0);
  graphColors[3] = color(62, 12, 232);
  graphColors[4] = color(13, 255, 243);
  graphColors[5] = color(200, 46, 232);
  graphColors[6] = color(200, 46, 0);
  graphColors[7] = color(200, 75, 232);
  graphColors[8] = color(200, 122, 232);
  graphColors[9] = color(200, 46, 232);
  graphColors[10] = color(125, 46, 232);  
  graphColors[11] = color(125, 122, 232);

  // settings save file
  topSketchPath = sketchPath();
  plotterConfigJSON = loadJSONObject(topSketchPath+"/plotter_config.json");

  // gui
  cp5 = new ControlP5(this);
  
  control_inputs[0] = "0.0"; // thrust
  control_inputs[1] = "0.0"; // setpoint
  control_inputs[2] = getPlotterConfigString("Kp");
  control_inputs[3] = getPlotterConfigString("Ki");
  control_inputs[4] = getPlotterConfigString("Kd");
  
  // init charts
  setChartSettings();
  for (int i=0; i<barChartValues.length; i++) {
    barChartValues[i] = 0;
  }
  // build x axis values for the line graph
  for (int i=0; i<lineGraphValues.length; i++) {
    for (int k=0; k<lineGraphValues[0].length; k++) {
      lineGraphValues[i][k] = 0;
      if (i==0)
        lineGraphSampleNumbers[k] = k;
    }
  }
  
  // start serial communication
  if (!mockupSerial) {
    //String serialPortName = Serial.list()[3];
    //serialPort = new Serial(this, serialPortName, 115200);
    println(Serial.list()); // Use this to print connected serial devices
    serialPort = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
    
  }
  else {
    serialPort = null;
  }
  
  // build the gui
  int x = 170;
  int y = 60;
  
  cp5.addTextfield("bcMaxY").setPosition(x, y).setText(getPlotterConfigString("bcMaxY")).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bcMinY").setPosition(x, y=y+150).setText(getPlotterConfigString("bcMinY")).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMaxY").setPosition(x, y=y+130).setText(getPlotterConfigString("lgMaxY")).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMinY").setPosition(x, y=y+200).setText(getPlotterConfigString("lgMinY")).setWidth(40).setAutoClear(false);

  cp5.addTextlabel("on/off2").setText("on/off").setPosition(x=13, y=20).setColor(0);
  cp5.addTextlabel("multipliers2").setText("multipliers").setPosition(x=55, y).setColor(0);
  
  cp5.addTextfield("bc-thrust").setPosition(x=60, y=30).setText(getPlotterConfigString("bcMultiplier1")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bc-input").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier2")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bc-setpoint").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier3")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bc-output").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier4")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bc-p-term").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier5")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bc-i-term").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier6")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("bc-d-term").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier7")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("bcMultiplier8").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier8")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("bcMultiplier9").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier9")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);  
//  cp5.addTextfield("bcMultiplier10").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier10")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("bcMultiplier11").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier11")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("bcMultiplier12").setPosition(x, y=y+40).setText(getPlotterConfigString("bcMultiplier12")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);  
  
  cp5.addToggle("bcVisible1").setPosition(x=x-50, y=30).setValue(int(getPlotterConfigString("bcVisible1"))).setMode(ControlP5.SWITCH);
  cp5.addToggle("bcVisible2").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible2"))).setMode(ControlP5.SWITCH);
  cp5.addToggle("bcVisible3").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible3"))).setMode(ControlP5.SWITCH);
  cp5.addToggle("bcVisible4").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible4"))).setMode(ControlP5.SWITCH);
  cp5.addToggle("bcVisible5").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible5"))).setMode(ControlP5.SWITCH);
  cp5.addToggle("bcVisible6").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible6"))).setMode(ControlP5.SWITCH);
  cp5.addToggle("bcVisible7").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible7"))).setMode(ControlP5.SWITCH);
//  cp5.addToggle("bcVisible8").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible8"))).setMode(ControlP5.SWITCH);
//  cp5.addToggle("bcVisible9").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible9"))).setMode(ControlP5.SWITCH);
//  cp5.addToggle("bcVisible10").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible10"))).setMode(ControlP5.SWITCH);  
//  cp5.addToggle("bcVisible11").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible11"))).setMode(ControlP5.SWITCH);  
//  cp5.addToggle("bcVisible12").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("bcVisible12"))).setMode(ControlP5.SWITCH);    

  cp5.addTextlabel("label").setText("on/off").setPosition(x=13, y=320).setColor(0); //y+90
  cp5.addTextlabel("multipliers").setText("multipliers").setPosition(x=55, y).setColor(0);
  cp5.addTextfield("thrust").setPosition(x=60, y=y+10).setText(getPlotterConfigString("lgMultiplier1")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("input").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier2")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("setpoint").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier3")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("output").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier4")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("p-term").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier5")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("i-term").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier6")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
  cp5.addTextfield("d-term").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier7")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("lgMultiplier8").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier8")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("lgMultiplier9").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier9")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);
//  cp5.addTextfield("lgMultiplier10").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier10")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);  
//  cp5.addTextfield("lgMultiplier11").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier11")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);  
//  cp5.addTextfield("lgMultiplier12").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier12")).setColorCaptionLabel(0).setWidth(40).setAutoClear(false);    
  
  cp5.addToggle("lgVisible1").setPosition(x=x-50, y=320+10).setValue(int(getPlotterConfigString("lgVisible1"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[0]);
  cp5.addToggle("lgVisible2").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible2"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[1]);
  cp5.addToggle("lgVisible3").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible3"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[2]);
  cp5.addToggle("lgVisible4").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible4"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[3]);
  cp5.addToggle("lgVisible5").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible5"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[4]);
  cp5.addToggle("lgVisible6").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible6"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[5]);
  cp5.addToggle("lgVisible7").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible7"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[6]);
//  cp5.addToggle("lgVisible8").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible8"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[7]);
//  cp5.addToggle("lgVisible9").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible9"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[8]);
//  cp5.addToggle("lgVisible10").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible10"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[9]);  
//  cp5.addToggle("lgVisible11").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible11"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[10]);  
//  cp5.addToggle("lgVisible12").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible12"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[11]);    


  cp5.addToggle("motors_onoff").setPosition(x, y=620).setValue(false).setMode(ControlP5.SWITCH);
  cp5.addTextfield("Thrust").setPosition(x+=45, y).setValue("0.0").setWidth(40).setAutoClear(false);
  cp5.addTextfield("setpoint_ac").setPosition(x+=45, y).setValue("0.0").setWidth(40).setAutoClear(false);
  cp5.addTextfield("Kp").setPosition(x+=45, y).setValue((getPlotterConfigString("Kp"))).setWidth(40).setAutoClear(false);
  cp5.addTextfield("Ki").setPosition(x+=45, y).setValue((getPlotterConfigString("Ki"))).setWidth(40).setAutoClear(false);
  cp5.addTextfield("Kd").setPosition(x+=45, y).setValue((getPlotterConfigString("Kd"))).setWidth(40).setAutoClear(false);  
  
  
}

byte[] inBuffer = new byte[100]; // holds serial message
int i = 0; // loop variable


void draw() {
 ///* Read serial and update values */
 // if (mockupSerial || serialPort.available() > 0) {
 //   String myString = "";
    
 //   for(int i=0;i<inBuffer.length;i++)
 //   {
 //     inBuffer[i] = '\0';
 //   }
    
 //   if (!mockupSerial) {
 //     try {
 //       serialPort.readBytesUntil('\r', inBuffer);
 //       //serialPort.clear();
 //     }
 //     catch (Exception e) {
 //       print(e);
 //     }
 //     myString = new String(inBuffer);
 //   }
 //   else {
 //     myString = mockupSerialFunction();
 //   }

 //   println(myString);

 //   String firstLetter = String.valueOf(myString.charAt(0));
 //   //if(firstLetter.equals(" ")) return ;
 //   if(firstLetter.equals("#")) return ;
 //   if(firstLetter.equals("\r")) return ;
 //   if(firstLetter.equals("\n")) return ;
 //   if(myString.length() == 0) return;
    
 //   // split the string at delimiter (space)
 //   String[] nums = split(myString, ' ');
    
 //   // count number of bars and line graphs to hide
 //   int numberOfInvisibleBars = 0;
 //   for (i=0; i<max_data_points; i++) {
 //     if (int(getPlotterConfigString("bcVisible"+(i+1))) == 0) {
 //       numberOfInvisibleBars++;
 //     }
 //   }
 //   int numberOfInvisibleLineGraphs = 0;
 //   for (i=0; i<max_data_points; i++) {
 //     if (int(getPlotterConfigString("lgVisible"+(i+1))) == 0) {
 //       numberOfInvisibleLineGraphs++;
 //     }
 //   }
 //   // build a new array to fit the data to show
 //   barChartValues = new float[max_data_points-numberOfInvisibleBars];

 //   // build the arrays for bar charts and line graphs
 //   int barchartIndex = 0;
 //   for (i=0; i<nums.length; i++) {

 //     // update barchart
 //     try {
 //       if (int(getPlotterConfigString("bcVisible"+(i+1))) == 1) {
 //         if (barchartIndex < barChartValues.length)
 //           barChartValues[barchartIndex++] = float(nums[i])*float(getPlotterConfigString("bcMultiplier"+(i+1)));
 //       }
 //       else {
 //       }
 //     }
 //     catch (Exception e) {
 //       print(e);
 //     }

 //     // update line graph
 //     try {
 //       if (i<lineGraphValues.length) {
 //         for (int k=0; k<lineGraphValues[i].length-1; k++) {
 //           lineGraphValues[i][k] = lineGraphValues[i][k+1];
 //         }

 //         lineGraphValues[i][lineGraphValues[i].length-1] = float(nums[i])*float(getPlotterConfigString("lgMultiplier"+(i+1)));
 //       }
 //     }
 //     catch (Exception e) {
 //         print(e);
 //     }
 //   }
 // }

  // draw the bar chart
  background(255); 
  BarChart.DrawAxis();              
  BarChart.Bar(barChartValues); // This draws a bar graph of Array4

  // draw the line graphs
  LineGraph.DrawAxis();
  for (int i=0;i<lineGraphValues.length; i++) {
    LineGraph.GraphColor = graphColors[i];
    if (int(getPlotterConfigString("lgVisible"+(i+1))) == 1)
      LineGraph.LineGraph(lineGraphSampleNumbers, lineGraphValues[i]);
  }     //<>//
}

void serialEvent(Serial serialPort) { 
    String myString = "";
    for(int i=0;i<inBuffer.length;i++)
    {
      inBuffer[i] = '\0';
    }
    
    if (!mockupSerial) {
      try {
        serialPort.readBytesUntil('\r', inBuffer);
        //serialPort.clear();
      }
      catch (Exception e) {
        print(e);
      }
      myString = new String(inBuffer);
    }
    else {
      myString = mockupSerialFunction();
    }

    String firstLetter = String.valueOf(myString.charAt(0));
    if(firstLetter.equals(" ")) return ;
    if(firstLetter.equals("\r")) return ;
    if(firstLetter.equals("\n")) return ;
    if(myString.length() == 0) return;
    
    //println(myString);
    if(firstLetter.equals("#")) return ;    
    
    // split the string at delimiter (space)
    String[] nums = split(myString, ' ');
    
    // count number of bars and line graphs to hide
    int numberOfInvisibleBars = 0;
    for (i=0; i<max_data_points; i++) {
      if (int(getPlotterConfigString("bcVisible"+(i+1))) == 0) {
        numberOfInvisibleBars++;
      }
    }
    int numberOfInvisibleLineGraphs = 0;
    for (i=0; i<max_data_points; i++) {
      if (int(getPlotterConfigString("lgVisible"+(i+1))) == 0) {
        numberOfInvisibleLineGraphs++;
      }
    }
    // build a new array to fit the data to show
    barChartValues = new float[max_data_points-numberOfInvisibleBars];

    // build the arrays for bar charts and line graphs
    int barchartIndex = 0;
    for (i=0; i<nums.length; i++) {

      // update barchart
      try {
        if (int(getPlotterConfigString("bcVisible"+(i+1))) == 1) {
          if (barchartIndex < barChartValues.length)
            barChartValues[barchartIndex++] = float(nums[i])*float(getPlotterConfigString("bcMultiplier"+(i+1)));
        }
        else {
        }
      }
      catch (Exception e) {
        print(e);
      }

      // update line graph
      try {
        if (i<lineGraphValues.length) {
          for (int k=0; k<lineGraphValues[i].length-1; k++) {
            lineGraphValues[i][k] = lineGraphValues[i][k+1];
          }

          lineGraphValues[i][lineGraphValues[i].length-1] = float(nums[i])*float(getPlotterConfigString("lgMultiplier"+(i+1)));
        }
      }
      catch (Exception e) {
          print(e);
      }
    }

} 

// called each time the chart settings are changed by the user 
void setChartSettings() {
  BarChart.xLabel=" Readings ";
  BarChart.yLabel="Value";
  BarChart.Title="";  
  BarChart.xDiv=1;  
  BarChart.yMax=int(getPlotterConfigString("bcMaxY")); 
  BarChart.yMin=int(getPlotterConfigString("bcMinY"));

  LineGraph.xLabel=" Samples ";
  LineGraph.yLabel="Value";
  LineGraph.Title="";  
  LineGraph.xDiv=20;  
  LineGraph.xMax=0; 
  LineGraph.xMin=-100;  
  LineGraph.yMax=int(getPlotterConfigString("lgMaxY")); 
  LineGraph.yMin=int(getPlotterConfigString("lgMinY"));
}

// handle gui actions
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isAssignableFrom(Textfield.class) || theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class)) {
    String parameter = theEvent.getName();
    String value = "";
    if (theEvent.isAssignableFrom(Textfield.class))
    {
      value = theEvent.getStringValue();
      
      int foo = 0;
      if( parameter.equals("Thrust") ) {
        control_inputs[0] = value;
        foo = 1;
      } else if ( parameter.equals("Kp") ) {
        control_inputs[2] = value;
        foo = 1;       
      } else if (parameter.equals("Ki") ) {
        control_inputs[3] = value;
        foo = 1;        
      } else if (parameter.equals("Kd") ) {
        control_inputs[4] = value;
        foo = 1;
      } else if (parameter.equals("setpoint_ac") ) {
        control_inputs[1] = value;
        foo = 1;        
      }
     
      if( foo == 1 ) {
        
        String foo1 = "";
        if( motors_on )
        {
          foo1 = control_inputs[0]+","+control_inputs[1]+","+control_inputs[2]+","+control_inputs[3]+","+control_inputs[4]+"\r\n";
        } else {
          foo1 = "0.0,"+control_inputs[1]+","+control_inputs[2]+","+control_inputs[3]+","+control_inputs[4]+"\r\n";          
        }
         
         print(foo1);
         
         if(serialPort != null)
           serialPort.write(foo1);
      }
    }
    else if (theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class))
    {
      value = theEvent.getValue()+"";
      
      if (parameter.equals("motors_onoff") ) {
        
        motors_on = (value.equals("1.0") ? true : false);
        
        print(motors_on); 
        print("\r\n");
        
        String foo1 = "";
        
        if( motors_on )
        {
          foo1 = control_inputs[0]+","+control_inputs[1]+","+control_inputs[2]+","+control_inputs[3]+","+control_inputs[4]+"\r\n";
        } else {
          foo1 = "0.0,"+control_inputs[1]+","+control_inputs[2]+","+control_inputs[3]+","+control_inputs[4]+"\r\n";          
        }
 
        print(foo1);   
 
        if(serialPort != null)
          serialPort.write(foo1);
        
      }
    }

    plotterConfigJSON.setString(parameter, value);
    saveJSONObject(plotterConfigJSON, topSketchPath+"/plotter_config.json");
  }
  setChartSettings();
}

// get gui settings from settings file
String getPlotterConfigString(String id) {
  String r = "";
  try {
    r = plotterConfigJSON.getString(id);
  } 
  catch (Exception e) {
    r = "";
  }
  return r;
}