import processing.serial.*;
Serial serial;

/*
String stringGyroX, stringGyroY;
String stringAccX, stringAccY;
String stringCompX, stringCompY;
String stringKalmanX, stringKalmanY;
*/
String stringthrust, stringinput_ypr_ac;
String stringsetpoint_ac,  stringoutput_ac;
String stringva, stringvc;
String stringac_pid_pterm;
String stringac_pid_iterm;
String stringac_pid_dterm;

final int width = 800;
final int height = 600;

float[] thrust = new float[width];
float[] input_ypr_ac = new float[width];

float[] setpoint_ac = new float[width];
float[] output_ac = new float[width];

float[] va = new float[width];
float[] vc = new float[width];

float[] ac_pid_pterm = new float[width];
float[] ac_pid_iterm = new float[width];
float[] ac_pid_dterm = new float[width];

boolean drawValues  = false;

void setup() {
  size(800, 600);
  println(Serial.list()); // Use this to print connected serial devices
  serial = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
  serial.bufferUntil('\n'); // Buffer until line feed

  for (int i = 0; i < width; i++) { // center all variables
    thrust[i] = height/2;
    input_ypr_ac[i] = height/2;
    setpoint_ac[i] = height/2;
    output_ac[i] = height/2;
    va[i] = height/2;
    vc[i] = height/2;
    ac_pid_pterm[i] = height/2;
    ac_pid_iterm[i] = height/2;
    ac_pid_dterm[i] = height/2;    
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
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
}

void serialEvent (Serial serial) {
  // Get the ASCII strings:
  stringthrust = serial.readStringUntil('\t');
  stringinput_ypr_ac = serial.readStringUntil('\t');
  stringsetpoint_ac = serial.readStringUntil('\t');
  stringoutput_ac = serial.readStringUntil('\t');
  stringva = serial.readStringUntil('\t');
  stringvc = serial.readStringUntil('\t');
  stringac_pid_pterm = serial.readStringUntil('\t');
  stringac_pid_iterm = serial.readStringUntil('\t');
  stringac_pid_dterm = serial.readStringUntil('\n');
 
  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph

  //printAxis(); // Used for debugging
}

void printAxis() {
  print(stringthrust);
  print(stringinput_ypr_ac);
  print(stringsetpoint_ac);
  print(stringoutput_ac);

  print('\t');

  print(stringva);
  print(stringvc);
  print(stringac_pid_pterm);
  print(stringac_pid_iterm);
  print(stringac_pid_dterm);  

  println();
}