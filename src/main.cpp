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
  pid.Init(0.0, 0.0, 0.0);
  pid.p_error = 0.0;
  pid.i_error = 0.0;
  pid.prev_cte = 0.0;

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

          // update p_error, i_error, d_error given the new crosstrack error
          // thoeses errors won't be changed in the TWIDDLE loop
          pid.UpdateError(cte);

          // create vector K = [Kp, Ki, Kd] and dK = [d_Kp, d_Ki, d_Kd]
          // in order to use for loop insread of apply twiddle to the 3 coefficients respectively
          std::vector<double> K{pid.Kp, pid.Ki, pid.Kd};
          std::vector<double> dK{1,1,1};

          // set the initialization of the best error as
          // the prediction of the cte after a unit time
          // with the steering angle unchanged
          double best_error = fabs(cte + speed * sin(angle));

          double tol = 0.2;
          int it = 0;
          // TWIDDLE loop here
          while((dK[0] + dK[1] + dK[2]) > tol) {
            //std::cout << "Iterations " << it << ", best error = " << best_error << std::endl;
            for(int i=0; i<K.size(); ++i) {

              K[i] += dK[i];
              pid.Init(K[0], K[1], K[2]);
              double error = fabs(pid.TotalError());
              if(error == 0) {
                break;
              }

              if(error < best_error) {
                best_error = error;
                dK[i] *= 1.1;
              }
              else{
                K[i] -= 2 * dK[i];
                pid.Init(K[0], K[1], K[2]);
                error = fabs(pid.TotalError());

                if (error < best_error) {
                  best_error = error;
                  dK[i] *= 1.1;
                }
                else{
                  K[i] += dK[i];
                  pid.Init(K[0], K[1], K[2]);
                  dK[i] *= 0.9;
                }
              }
            }  // for loop end
            it += 1;
          }
          //std::cout << "After " << it << " Iterations:" << std::endl;
          //std::cout << "K = [" << K[0] << ",\t" << K[1] << ",\t" << K[0] << "]" << std::endl;

          steer_value = angle - pid.TotalError();
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
