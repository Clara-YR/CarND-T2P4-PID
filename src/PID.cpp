#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle = false) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    i_error = 0;
    prev_cte = 0;
    dt = 1;
    twiddle_done = true;

    if(twiddle) {
        // for twiddle
        dKp = 1.0;
        dKi = 0.1;
        dKd = 1.0;
        n_run = 3000;
        cte_counter = 0;
        // twiddle_step = 10 * discuss_case + i_th
        twiddle_step = 0;
        sum_cte2 = 0;
        best_error = -1;  // use to judge whether the best_value has been assigned
        //run_th = 1;
        twiddle_done = false;
    }
}

void PID::UpdateError(double cte, double speed) {
    // take the mean cte from t to t+dt as the cte at time t
    //double cte_ = cte + speed * sin(angle) * dt/2;

    // use cte to calculate p_error
    p_error = Kp * cte;
    // use cte_ to calculate i_error and d_error
    i_error += Ki * cte * dt;
    d_error = Kd * (cte - prev_cte) / dt;

    if(!twiddle_done) {
        cte_counter += 1;
        sum_cte2 += cte * cte;
        if(cte_counter > 100 && speed < 0.5) {
            sum_cte2 += (n_run - cte_counter);
            cte_counter = n_run;
        }
    }
}

double PID::TotalError() {
    double total_error = p_error + i_error + d_error;
    return total_error;
}

void PID::Twiddle() {

}

