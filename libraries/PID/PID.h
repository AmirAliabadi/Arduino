#ifndef PID_h
#define PID_h

#include <stdlib.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "pins_arduino.h"

class PID {
	public:
	
	PID(float p, float i, float d, float ref, float in, float maxCtrl, float minCtrl) {
		Kp = p;
		Ki = i;
		Kd = d;
		prevTime = millis();
		prevRef = ref;
		prevInput = in;
		
		minLimit = minCtrl;
		maxLimit = maxCtrl;
		
		iTerm = 0;
	}
	
	void resetITerm() {
		iTerm = 0;
		prevTime = millis();
	}

	float calculate(float ref, float input) {
		// Calculate sampling time
		long dt = (millis() - prevTime); // Convert to seconds
		float dt_float = dt * 0.001 ;
		
		float error = ref - input;
		pTerm = Kp * (ref - input);
		dTerm = -Kd * (input - prevInput)/dt_float; // dError/dt = - dInput/dt
		iTerm += Ki * error * dt;
		
		// Calculate control
		float output = pTerm + iTerm + dTerm;
		
		// Anti-windup
		if (output > maxLimit) {
			iTerm -= output - maxLimit;
			output = maxLimit;
		} else if ( output < minLimit ){
			iTerm += minLimit - output;
			output = minLimit;
		} else {
			//output is output
		}
		
		// Update state
		prevTime = millis();
		prevRef = ref;
		prevInput = input;
		
		return output;
	}

	void setControlCoeffs(int* pidVector) {
		Kp = pidVector[0];
		Ki = pidVector[1];
		Kd = pidVector[2];
	}
	
	private:
	long prevTime;
	float prevRef;
	float prevInput;
	float pTerm;
	float iTerm;
	float dTerm;
	float minLimit;
	float maxLimit;
	float Kp, Ki, Kd;
};

#endif