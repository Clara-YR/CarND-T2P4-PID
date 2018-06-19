
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // initial all pid variables to 0
  pid.Init(4, 0.002, 0.01, false);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          //std::cout << "CTE = " << cte << std::endl;
          // update pid error with new cte input
          pid.UpdateError(cte, speed);
          // calcualte new steer angle
          steer_value = - pid.TotalError();
          //double throttle_value = 0.15 * (1 + cos(deg2rad(angle)))/2;
          double throttle_value = 0.2;
          //std::cout << "CTE Counter = " << pid.cte_counter << std::endl;

          /*/---------------------------------------------------------------//
          // Using TWIDDLE Method to find the best coefficients Kp, Ki, Kd //
          //---------------------------------------------------------------//
          if(pid.cte_counter == pid.n_run && !pid.twiddle_done) {

            //std::cout << "1000 iteration Run" << std::endl;
            // calculate the mean of the cte square
            double error = pid.sum_cte2 / pid.n_run;

            // INITIALIZE best_error
            if(pid.best_error==-1) {

              // initialize the best error with the 1st run
              pid.best_error = error;
              std::cout << "Initialize best_error = " << pid.best_error << std::endl;

              // K[0] --> K[0]+dK[0], run simulator to get error(K+dK)
              pid.Kp += pid.dKp;
              std::cout << "\tK[0] = 0 -->> K[0] + dK[0] = 1" << std::endl;
              // next discuss case 1 of K[0] and dK[0]
              pid.twiddle_step = 10;

            }

            // DISCUSS whether error < best_error
            else{
              std::vector<double> K{pid.Kp, pid.Ki, pid.Kd};
              std::vector<double> dK{pid.dKp, pid.dKi, pid.dKd};

              // twiddle_step = discuss_case * 10 + i_th
              int discuss_case = (int) pid.twiddle_step / 10;
              int i_th = pid.twiddle_step % 10;

              // whether error(K[i]+dK[i]) < best_error
              if(discuss_case == 1){

                std::cout << "When error = " << error << ":\n\tK[" << i_th << "] + dK[" << i_th << "] = " << K[i_th];
                if(error < pid.best_error) {
                  std::cout << " \t\tSUCCESS !!!" << std::endl;
                  // update best error for pid
                  pid.best_error = error;

                  // update dK to 1.1*dK
                  std::cout << "\tdK[" << i_th << "] = " << dK[i_th];
                  dK[i_th] *= 1.1;
                  std::cout << " -->> 1.1*dK[" << i_th << "] = " << dK[i_th] << std::endl;

                  // K[i] --> K[i]+1.1*dK[i]
                  std::cout << "\tK[" << i_th << "] = " << K[i_th];
                  K[i_th] += dK[i_th];
                  std::cout << " -->> K[" << i_th << "] + 1.1dK[" << i_th <<"] = " << K[i_th] << std::endl;
                  // still discuss case 1 for K[i] and dK[i]
                  // no need to change pid.twiddle_step
                  //pid.twiddle_step = 1*10 + i_th;  // twiddle_step = discuss_case * 10 + i_th

                  // change i_th from 0/1/2 to (1/2/3)%3 = (1/2/0)
                  ///int i_next = (i_th + 1) % 3;
                  // K[next] --> dK[next], run simulator to get error(K+dK)
                  ///std::cout << "\tK[" << i_next << "] + dK[" << i_next << "] = " << K[i_next];
                  //K[i_next] += dK[i_next];
                  ///std::cout << "+ " << dK[i_next] << " = " << K[i_next] << std::endl;
                  // next discuse case 1 of next item of K and dK
                  ///pid.twiddle_step = 1*10 + i_next;  // twiddle_step = discuss_case * 10 + i_th

                }
                else {
                  std::cout << " FAIL" << std::endl;

                  // K[i]+dK[i] --> K[i]-dK[i], run simulator to get error()
                  std::cout << "\tK[" << i_th << "] + dK[" << i_th << "] = " << K[i_th];
                  K[i_th] -= 2 * dK[i_th];
                  std::cout << " -->> K[" << i_th << "] - dK[" << i_th << "] = " << K[i_th] << std::endl;
                  // next discuss case 2 for K[i] and dK[i]
                  pid.twiddle_step = 2*10 + i_th;  // twiddle_step = discuss_case * 10 + i_th
                }
              }

              // whether error(K[i]-dK[i]) < best_error
              else if(discuss_case == 2){

                std::cout << "When error = " << error << ":\n\tK[" << i_th << "] - dK[" <<i_th << "] = " << K[i_th];
                if(error < pid.best_error) {
                  std::cout << " \t\tSUCCESS !!!" << std::endl;
                  // update best error for pid
                  pid.best_error = error;

                  // update dK to 1.1*dK
                  std::cout << "\tdK[" << i_th << "] = " << dK[i_th];
                  dK[i_th] *= 1.1;
                  std::cout << " -->> 1.1*dK[" << i_th << "] = " << dK[i_th] << std::endl;

                  // K[i] --> K[i]+1.1*dK[i]
                  std::cout << "\tK[" << i_th << "] = " << K[i_th];
                  K[i_th] += dK[i_th];
                  std::cout << " -->> K[" << i_th << "] + 1.1dK[" << i_th <<"] = " << K[i_th] << std::endl;
                  // still discuss case 1 for K[i] and dK[i]
                  // no need to change pid.twiddle_step

                  // change i_th from 0/1/2 to (1/2/3)%3 = (1/2/0)
                  /// int i_next = (i_th + 1) % 3;
                  // K[next] --> dK[next], run simulator to get error(K+dK)
                  ///std::cout << "\tK[" << i_next << "] + dK[" << i_next << "] = " << K[i_next];
                  ///K[i_next] += dK[i_next];
                  ///std::cout << " + " << dK[i_next] << " = " << K[i_next] << std::endl;
                  // next discuse case 1 of next item of K and dK
                  ///pid.twiddle_step = 1*10 + i_next;  // twiddle_step = discuss_case * 10 + i_th

                }
                else{
                  std::cout << " FAIL" << std::endl;

                  // K[i]-dK[i] --> K[i]
                  std::cout << "\tK[" << i_th << "] - dK[" << i_th << "] = " << K[i_th];
                  K[i_th] += dK[i_th];
                  std::cout << " -->> K[" << i_th << "] = " << K[i_th] << std::endl;

                  // update dK[i] --> 0.9*dK[i]
                  std::cout << "\tdK[" << i_th << "] = " << dK[i_th];
                  dK[i_th] *= 0.9;
                  std::cout << " -->> 0.9*dK[" << i_th << "] = " << dK[i_th] << std::endl;

                  if (dK[i_th] > 0.6 * K[i_th]) {
                    // both K+dK and K-dK fail, but dK > K*0.1, keep tuning K[i_th]
                    // K[i] --> K[i]+0.9*dK[i]
                    std::cout << "\tK[" << i_th << "] = " << K[i_th];
                    K[i_th] += dK[i_th];
                    std::cout << " -->> K[" << i_th << "] + 0.9dK[" << i_th <<"] = " << K[i_th] << std::endl;
                    // next discuss case 1 for k[i] and dK[i]
                    pid.twiddle_step = 1*10 + i_th;  // twiddle_step = discuss_case * 10 + i_th
                  }
                  else {
                    // both K+dK and K-dK fail, but dK <= K*0.1, turn to tune K[i_next]
                    std::cout << "Complete K[" << i_th << "] tuning !" << std::endl;
                    // change i_th from 0/1/2 to (1/2/3)%3 = (1/2/0)
                    int i_next = (i_th + 1) % 3;
                    // K[next] --> dK[next], run simulator to get error(K+dK)
                    std::cout << "\tK[" << i_next << "] + dK[" << i_next << "] = " << K[i_next] << " + ";
                    K[i_next] += dK[i_next];
                    std::cout << dK[i_next] << " = " << K[i_next] << std::endl;
                    // next discuse case 1 of next item of K and dK
                    pid.twiddle_step = 1*10 + i_next;  // twiddle_step = discuss_case * 10 + i_th
                  }
                }
              }
              // NEW RUN start a new simulatrion with 100 timesteps
              // update Kp, Ki, Kd to start a new 100 iteration run
              pid.Kp = K[0];
              pid.Ki = K[1];
              pid.Kd = K[2];
              std::cout << "Kp = " << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd << std::endl;
              pid.dKp = dK[0];
              pid.dKi = dK[1];
              pid.dKd = dK[2];
              std::cout << "dKp = " << pid.dKp << ", dKi = " << pid.dKi << ", dKd = " << pid.dKd << std::endl;
            }

            // reset the cte counter and sum of cte*ct
            pid.cte_counter = 0;
            pid.sum_cte2 = 0;
            // restart the simulator
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }

          //----------------------------------------------------------//
          // Set Kp, Ki, Kd to the found values to run PID Controller //
          //----------------------------------------------------------//
          if((0.6 * pid.dKp < pid.Kp) && (0.6 * pid.dKi < pid.Ki) && (0.6 * pid.dKd < pid.Kd)) {
            std::cout << "Twiddle Done, Kp =" << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd << std::endl;
            pid.twiddle_done = true;
          }*/


          // DEBUG
          ///std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ///std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
