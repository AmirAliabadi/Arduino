boolean locked = false;
boolean overBox = true;
float bx,by;
float yOffset = 0.0;
float xOffset = 0.0;

float ac_setpoint_last = 0.0;
float bd_setpoint_last = 0.0;

float ac_setpoint = 0.0;
float bd_setpoint = 0.0;


void mouseWheel(MouseEvent event) {
  //float e = event.getCount();
  //if( e <0.0 ) cur_throttle += 10; //  (int)Input_Thrust;
  //else if( e >0.0 ) cur_throttle -= 10; //  (int)Input_Thrust;
  
  //if(cur_throttle < 900.0) Send_To_Arduino2(0.0, cur_throttle);        
  //if(cur_throttle > 0.0) Send_To_Arduino2(0.0, cur_throttle);           
}

void mousePressed() {
  //if(overBox) { 
  //  locked = true; 
  //} else {
  //  locked = false;
  //}
  //xOffset = mouseX;//-bx; 
  //yOffset = mouseY;//-by; 
}

void mouseDragged() {
  //if(locked) {
  //  bx = mouseX-xOffset; 
  //  by = mouseY-yOffset; 
    
  //  ac_setpoint = round(constrain(bx/15.0,-20.0,20.0));
  //  bd_setpoint = round(constrain(by/15.0,-20.0,20.0));
    
  //  if( ac_setpoint != ac_setpoint_last) {
  //   // println(ac_setpoint);
  //    Send_To_Arduino2(14.0, ac_setpoint);
  //  }
  //  if( bd_setpoint != bd_setpoint_last) {
  //  //  println(bd_setpoint);
  //    Send_To_Arduino2(15.0, bd_setpoint);
  //  }
    
  //  ac_setpoint_last = ac_setpoint;
  //  bd_setpoint_last = bd_setpoint;
    
  //  //SPField.setValue(constrain(bx/20.0,-20,20));
  //}
}

void mouseReleased() {
  //locked = false;
  //ac_setpoint = 0.0;
  //bd_setpoint = 0.0;  
  
  //Send_To_Arduino2(14.0, ac_setpoint);
  //Send_To_Arduino2(15.0, bd_setpoint); 

  //ac_setpoint_last = ac_setpoint;
  //bd_setpoint_last = bd_setpoint;
}