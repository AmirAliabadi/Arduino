import processing.serial.*;
Serial serial;

final int width = 800;
final int height = 600;


/*
String stringGyroX, stringGyroY;
String stringAccX, stringAccY;
String stringCompX, stringCompY;
String stringKalmanX, stringKalmanY;
*/
//String stringthrust, stringinput_ypr_ac;
//String stringsetpoint_ac,  stringoutput_ac;
//String stringva, stringvc;
//String stringac_pid_pterm;
//String stringac_pid_iterm;
//String stringac_pid_dterm;


//float[] thrust = new float[width];
//float[] input_ypr_ac = new float[width];

//float[] setpoint_ac = new float[width];
//float[] output_ac = new float[width];

//float[] va = new float[width];
//float[] vc = new float[width];

//float[] ac_pid_pterm = new float[width];
//float[] ac_pid_iterm = new float[width];
//float[] ac_pid_dterm = new float[width];

boolean drawValues  = false;

int[][] displayBuffer = new int[25][width];

void setup() {
  size(800, 600);
  
  println(Serial.list()); // Use this to print connected serial devices
  serial = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
  serial.bufferUntil('\n'); // Buffer until line feed

  for (int i = 0; i < 25; i++) {
    for (int j = 0; j < width; j++) {
      displayBuffer[i][j] = height/2;
    }
  }

  drawGraph(); // Draw graph at startup
}

void draw() {
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    drawGraph();
  }
}

void drawGraph() {
  background(255); // White
  for (int i = 0; i < width; i++) {
    stroke(200); // Grey
    line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(0); // Black
  for (int i = 1; i <= 3; i++)
  {
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg
  }

  //convert();
  //drawAxisX();
  //drawAxisY();
  
  drawIt(); 
}

void serialEvent (Serial serial) {

  readCsvToVector();  
 
  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph
  
  //drawGraph();   

}

void readCsvToVector() 
{
  int i = 0;
  while(i<25) {
    String foo = serial.readStringUntil('\t');

    if(foo == null) break;
    
    foo = trim(foo);
   
    displayBuffer[i][displayBuffer[i].length - 1] = int(foo); // map(float(foo), 0, 200, 0, height);
    
    print(displayBuffer[i][displayBuffer[i].length - 1]); print('\t');    
    
    i++;
  }
  
  println();  
}