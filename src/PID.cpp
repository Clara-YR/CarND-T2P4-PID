#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double new_Kp, double new_Ki, double new_Kd) {
    Kp = new_Kp;
    Ki = new_Ki;
    Kd = new_Kd;
}

void PID::UpdateError(double cte) {


    // proportiional error
    p_error = Kp * cte ;
    // integral error
    i_error = Ki * int_cte;
    // derivative error
    d_error = Kd * diff_cte;
}

double PID::TotalError() {

    return p_error + i_error + d_error;
}

