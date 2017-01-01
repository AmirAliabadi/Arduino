#ifndef PID_h
#define PID_h

#include <stdlib.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "pins_arduino.h"

class MyPID {
	public:
	
	MyPID(float p, float i, float d, float ref, float in, float maxCtrl, float minCtrl) {
		Kp = p;
		Ki = i;
		Kd = d;
		prevTime = micros();
		prevRef = ref;
		prevInput = in;
		
		minLimit = minCtrl;
		maxLimit = maxCtrl;
		
		iTerm = 0;
	}
	
	void resetITerm() {
		iTerm = 0;
		prevTime = micros();
	}

	float calculate(float ref, float input, bool debug) {
		// Calculate sampling time
		long dt = (micros() - prevTime); // Convert to seconds
		if( dt > 500 ) {
		float dt_float = (float)dt ;//* 0.001 ;
		
		float error = ref - input;
		
		pTerm  =  Kp * (ref - input);
		iTerm +=  Ki/1000.0 * (error * dt);
		dTerm  = -Kd * (input - prevInput)/dt_float; // dError/dt = - dInput/dt	

		if( iTerm >  75.0 ) iTerm =  75.0;
		if( iTerm < -75.0 ) iTerm = -75.0;
		
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
		prevTime = micros();
		prevRef = ref;
		prevInput = input;
		last_output = output;
		}
		
		return last_output;
	}

	void setControlCoeffs(float* pidVector) {
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
	float last_output;
};

#endif