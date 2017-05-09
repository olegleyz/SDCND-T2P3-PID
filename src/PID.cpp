#include "PID.h"
#include <uWS/uWS.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/



PID::PID() {}
PID::~PID() {}

void PID::Init(double Kp_a, double Ki_a, double Kd_a) {
  Kp = Kp_a;
  Kd = Kd_a;
  Ki = Ki_a;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  numSteps = 0;
  bestErr = 0;
  twiddleFinish = false;
}


void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  numSteps++;

}


double PID::TotalError() {

  double totalError = p_error * Kp + d_error * Kd + i_error * Ki;
  if (totalError > 1) totalError = 1;
  if (totalError < -1) totalError = -1;
  return totalError;
}

void PID::Twiddle(double cte, uWS::WebSocket<uWS::SERVER> ws) {
  static double dpp = 1;
  static double dpi = 1;
  static double dpd = 1;

  static double best_Kp = 0;
  static double best_Ki = 0;
  static double best_Kd = 0;

  const int minStepsBeforeError = 100;
  const int numStepsError = 2000;
  static double err = 0;
  static double bestErr = std::numeric_limits<double>::max() - 10000;
  static bool error_flag = false;
  static bool twiddleAdd = true;
  static int coefNum = 0;
  static bool firstRun = true;
  static bool restart = false;
  static int iter = 0;



  if (cte > 2.5 || cte < -2.5) {
    restart = true;

  }

  if (numSteps  == minStepsBeforeError) {
    error_flag = true;
  } else if (numSteps == numStepsError || restart) {
   // cout << "restart: "<<restart<<endl;
    error_flag = false;

    if (restart) {
      err = err + (numStepsError - numSteps - minStepsBeforeError) * (2.5*2.5);
      err = bestErr + 1;
      restart = false;
    }

    err = err / (numStepsError - minStepsBeforeError);
    if (firstRun == true) {
      bestErr = err;
      best_Kp = Kp;
      best_Ki = Ki;
      best_Kd = Kd;
      firstRun = false;
      Kp += dpp;
    } else {
      cout << "iteation #: "<<iter<<endl;
      iter++;



      if (err < bestErr) {
        bestErr = err;
        best_Kp = Kp;
        best_Ki = Ki;
        best_Kd = Kd;
        coefNum == 0 ? dpp *= 1.1 : coefNum == 1 ? dpi *= 1.1 : dpd *= 1.1;
        //coefNum == 0 ? dpp *= (1.0+0.723733427) : coefNum == 1 ? dpi *= (1.0+0.723733427) : dpd *= (1.0+0.723733427);

        coefNum = (coefNum + 1) % 3;
        coefNum == 0 ? Kp += dpp : coefNum == 1 ? Ki += dpi : Kd += dpd;
        } else {
        if (twiddleAdd) {
          coefNum == 0 ? Kp -= 2 * dpp : coefNum == 1 ? Ki -= 2 * dpi : Kd -= 2 * dpd;
          twiddleAdd = false;
        } else {
          coefNum == 0 ? Kp += dpp : coefNum == 1 ? Ki += dpi : Kd += dpd;
          coefNum == 0 ? dpp *= 0.9 : coefNum == 1 ? dpi *= 0.9 : dpd *= 0.9;
          //coefNum == 0 ? dpp *= (1.0-0.723733427) : coefNum == 1 ? dpi *= (1.0-0.723733427) : dpd *= (1.0-0.723733427);
          coefNum = (coefNum + 1) % 3;
          twiddleAdd = true;
          coefNum == 0 ? Kp += dpp : coefNum == 1 ? Ki += dpi : Kd += dpd;
          if (dpp + dpi + dpd <= 0.05) {
            twiddleFinish = true;
            Kp = best_Kp;
            Ki = best_Ki;
            Kd = best_Kd;
        }
      }
    }
      cout <<"error: " << err << "; best error: "<<bestErr<<"; Kp: "<<Kp<<", Ki: "<<Ki<<", Kd: "<<Kd<<endl;
      cout << "dp: "<<dpp<<", "<<dpi<<", "<<dpd<<endl;
      cout <<"best coef so far: "<<best_Kp<<", "<<best_Ki<<", "<<best_Kd<<endl;
    }


    err = 0;
    Restart(ws);
    restart = false;

  }
  if (error_flag) {
    err += cte * cte;
  }


  //cout <<endl<<endl<<"numSteps: "<<numSteps<<", firstRun: "<<firstRun<<endl;
  ///cout << "error_flag: " <<error_flag<< "err: " << err << endl;
  //cout << "Initial coef: 0.225, 0.001, 16.0" << endl;
  //cout<<bestErr<<" sum(dp): "<<dpp+dpi+dpd<<" Kp: "<<Kp<<", Ki: "<<Ki<<" , Kd: "<<Kd<<endl<<endl;

}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  p_error = 0;
  i_error = 0;
  d_error = 0;
  numSteps = 0;
}

double PID::Tune(double cte, uWS::WebSocket<uWS::SERVER> ws) {
  static int numSteps = 0;
  static double err = 0;
  static double bestErr = 0;
  static double dpp = 1;
  static double dpi = 1;
  static double dpd = 1;


  // Step 0. Calculate initial error
  if (dpp+dpi+dpd < 0.2) {
    cout << "Final Kp: " << Kp << ", Ki: "<< Ki << ", Kd: " << Kd << endl << endl;
    return 2;
  }
  numSteps ++; //track how many steps have we done
  if (numSteps > 100 && numSteps < 200) {
    err += cte*cte;
  } else if (numSteps == 200) {
    bestErr = err / 100.0;
    err = 0;
    Restart(ws);
    Kp += dpp;

  } else if (numSteps>300 && numSteps < 400){
    err += cte * cte;
  } else if (numSteps == 400) {
    err = err / 100.0;
    if (err < bestErr) {
      bestErr = err;
      dpp *= 1.1;
    } else {
      Kp -= 2 * dpp;
      Restart(ws);

    }
  }


}

/*
 static double err = 0;
  if (numSteps==200) {
    numSteps = 0;
    bestErr = err / 100.0;
    err = 0;
  } else if (numSteps > 100){
    err += cte * cte;
  }
 */