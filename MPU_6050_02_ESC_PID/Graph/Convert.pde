//convert all axis
final int minAngle = -180;
final int maxAngle = 180;

void convert() {
  /* Convert the gyro x-axis */
  if (stringthrust != null) {
    stringthrust = trim(stringthrust); // Trim off any whitespace
    thrust[thrust.length - 1] = map(float(stringthrust), 0, 200, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the gyro y-axis */
  if (stringinput_ypr_ac != null) {
    stringinput_ypr_ac = trim(stringinput_ypr_ac); // Trim off any whitespace
    input_ypr_ac[input_ypr_ac.length - 1] = map(float(stringinput_ypr_ac), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the accelerometer x-axis */
  if (stringsetpoint_ac != null) {
    stringsetpoint_ac = trim(stringsetpoint_ac); // Trim off any whitespace
    setpoint_ac[setpoint_ac.length - 1] = map(float(stringsetpoint_ac), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the accelerometer y-axis */
  if (stringoutput_ac != null) {
    stringoutput_ac = trim(stringoutput_ac); // Trim off any whitespace
    output_ac[output_ac.length - 1] = map(float(stringoutput_ac), -255, 255, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the complementary filter x-axis */
  if (stringva != null) {
    stringva = trim(stringva); // Trim off any whitespace
    va[va.length - 1] = map(float(stringva), 0, 1500, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the complementary filter x-axis */
  if (stringvc != null) {
    stringvc = trim(stringva); // Trim off any whitespace
    vc[vc.length - 1] = map(float(stringvc), 0, 1500, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the kalman filter x-axis */
  if (stringac_pid_pterm != null) {
    stringac_pid_pterm = trim(stringac_pid_pterm); // Trim off any whitespace
    ac_pid_pterm[ac_pid_pterm.length - 1] = map(float(stringac_pid_pterm), -100, 100, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the kalman filter y-axis */
  if (stringac_pid_iterm != null) {
    stringac_pid_iterm = trim(stringac_pid_iterm); // Trim off any whitespace
    ac_pid_iterm[ac_pid_iterm.length - 1] = map(float(stringac_pid_iterm), -100, 100, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }
  
  if (stringac_pid_dterm != null) {
    stringac_pid_dterm = trim(stringac_pid_dterm); // Trim off any whitespace
    ac_pid_dterm[ac_pid_dterm.length - 1] = map(float(stringac_pid_dterm), -100, 100, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }  
}