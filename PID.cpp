#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error = 0;
    i_error = 0;
    d_error = 0;
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;
}

double PID::TotalError() {
    double sqTotalError;
    sqTotalError = (d_error*d_error + p_error*p_error + i_error*i_error);
    return(sqTotalError);
}


