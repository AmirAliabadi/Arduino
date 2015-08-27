void drawAxisX() {
  /* Draw gyro x-axis */
  noFill();
  stroke(255, 255, 0); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, thrust[0]);
  for (int i = 1; i < thrust.length; i++) {
    if ((thrust[i] < height/4 && thrust[i - 1] > height/4*3) || (thrust[i] > height/4*3 && thrust[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, thrust[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < thrust.length;i++)
    thrust[i-1] = thrust[i];

  /* Draw acceleromter x-axis */
  noFill();
  stroke(0, 255, 0); // Green
  // Redraw everything
  beginShape();
  vertex(0, input_ypr_ac[0]);
  for (int i = 1; i < input_ypr_ac.length; i++) {
    if ((input_ypr_ac[i] < height/4 && input_ypr_ac[i - 1] > height/4*3) || (input_ypr_ac[i] > height/4*3 && input_ypr_ac[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, input_ypr_ac[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < input_ypr_ac.length;i++)
    input_ypr_ac[i-1] = input_ypr_ac[i];

  /* Draw complementary filter x-axis */
  noFill();
  stroke(0, 0, 255); // Blue
  // Redraw everything
  beginShape();
  vertex(0, setpoint_ac[0]);
  for (int i = 1; i < setpoint_ac.length; i++) {
    if ((setpoint_ac[i] < height/4 && setpoint_ac[i - 1] > height/4*3) || (setpoint_ac[i] > height/4*3 && setpoint_ac[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, setpoint_ac[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < setpoint_ac.length; i++)
    setpoint_ac[i-1] = setpoint_ac[i];

  /* Draw kalman filter x-axis */
  noFill();
  stroke(255, 0, 0);// Red
  // Redraw everything
  beginShape();
  vertex(0, output_ac[0]);
  for (int i = 1; i < output_ac.length; i++) {
    if ((output_ac[i] < height/4 && output_ac[i - 1] > height/4*3) || (output_ac[i] > height/4*3 && output_ac[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, output_ac[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < output_ac.length; i++)
    output_ac[i-1] = output_ac[i];
}

void drawAxisY() {
  /* Draw gyro y-axis */
  noFill();
  stroke(255, 0, 255); // Purble
  // Redraw everything
  beginShape();
  vertex(0, va[0]);
  for (int i = 1; i < va.length; i++) {
    if ((va[i] < height/4 && va[i - 1] > height/4*3) || (va[i] > height/4*3 && va[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, va[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < va.length;i++)
   va[i-1] = va[i];

////
  /* Draw acceleromter y-axis */
  noFill();
  stroke(0, 124, 255); // Light blue
  // Redraw everything
  beginShape();
  vertex(0, vc[0]);
  for (int i = 1; i < vc.length; i++) {
    if ((vc[i] < height/4 && vc[i - 1] > height/4*3) || (vc[i] > height/4*3 && vc[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, vc[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < vc.length;i++)
    vc[i-1] = vc[i];
////
    

  /* Draw complementary filter y-axis */
  noFill();
  stroke(124, 252, 0); // Lawn Green
  // Redraw everything
  beginShape();
  vertex(0, ac_pid_pterm[0]);
  for (int i = 1; i < ac_pid_pterm.length; i++) {
    if ((ac_pid_pterm[i] < height/4 && ac_pid_pterm[i - 1] > height/4*3) || (ac_pid_pterm[i] > height/4*3 && ac_pid_pterm[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, ac_pid_pterm[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < ac_pid_pterm.length;i++)
    ac_pid_pterm[i-1] = ac_pid_pterm[i];

  /* Draw kalman filter y-axis */
  noFill();
  stroke(0, 0, 0); // Black
  // Redraw everything
  beginShape();
  vertex(0, ac_pid_iterm[0]);
  for (int i = 1; i < ac_pid_iterm.length; i++) {
    if ((ac_pid_iterm[i] < height/4 && ac_pid_iterm[i - 1] > height/4*3) || (ac_pid_iterm[i] > height/4*3 && ac_pid_iterm[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, ac_pid_iterm[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i<ac_pid_iterm.length;i++)
    ac_pid_iterm[i-1] = ac_pid_iterm[i];
    
///
  /* Draw kalman filter y-axis */
  noFill();
  stroke(0, 0, 0); // Black
  // Redraw everything
  beginShape();
  vertex(0, ac_pid_dterm[0]);
  for (int i = 1; i < ac_pid_dterm.length; i++) {
    if ((ac_pid_dterm[i] < height/4 && ac_pid_dterm[i - 1] > height/4*3) || (ac_pid_dterm[i] > height/4*3 && ac_pid_dterm[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, ac_pid_dterm[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i<ac_pid_dterm.length;i++)
    ac_pid_dterm[i-1] = ac_pid_dterm[i];
///
}