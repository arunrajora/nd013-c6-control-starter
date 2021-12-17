/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;

  outputLimitMax = output_lim_maxi;
  outputLimitMin = output_lim_mini;

  pError = 0.0;
  iError = 0.0;
  dError = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   dError = (cte - pError) / deltaTime;
   pError = cte;
   iError = cte * deltaTime;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = Kp * pError + Ki * iError +  Kd * dError;
   control = max(control, outputLimitMax);
   control = min(control, outputLimitMin);
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   deltaTime = new_delta_time;
   return deltaTime;
}