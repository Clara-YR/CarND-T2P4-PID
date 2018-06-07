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
  // initialize all the target parameters to 0.0
  //std::vector<double> K(3, 0.0);
  pid.Init(0.0, 0.0, 0.0);
  pid.prev_cte = 0;
  pid.int_cte = 0;
  pid.x = 0;
  pid.y = 0;
  pid.orientation = 0;

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
          std::cout << "cte = " << cte << std::endl;
          std::cout << "prev_cte = " << pid.prev_cte << std::endl;
          std::cout << "int_cte = " << pid.int_cte << std::endl;
          std::cout << "diff_cte = " << pid.diff_cte << std::endl;
          std::cout << "speed = " << speed << std::endl;
          std::cout << "angle = " << angle << std::endl;
          pid.x += speed * cos(pid.orientation);
          pid.y += speed * sin(pid.orientation);
          pid.orientation += angle;

          double best_err = cte;
          pid.diff_cte = cte - pid.prev_cte;
          pid.prev_cte = cte;
          pid.int_cte += cte;

          // take the error at present as the initial value of best error


          // create vectors to store the modified coefficients
          std::vector<double> dK(3, 1.0);

          std::vector<double> K{pid.Kp, pid.Ki, pid.Kd};

          // TWIDDLE to find the best Kp, Ki, Kd
          double tol = 0.2;  // 0.2
          int it = 0;

          while (it < 10) {
            std::cout << "Iteration " << it << ", best error = " << best_err << std::endl;
            for(int i=0; i<3; ++i) {
              K[i] += dK[i];
              pid.Init(K[0], K[1], K[2]);
              pid.UpdateError(cte);
              double err = pid.TotalError();

              if(err < best_err) {
                best_err = err;
                dK[i] *= 1.1;
                std::cout << "K+dK, dK*1.1, \t";
              } else {
                K[i] -= 2*dK[i];
                pid.UpdateError(cte);

                if(err < best_err) {
                  best_err = err;
                  dK[i] *= 1.1;
                  std::cout << "K-dK, dK*1.1, \t";
                } else {
                  K[i] += dK[i];
                  dK[i] *= 0.9;
                  std::cout << "dK*0.9, try K+/- dK again, \t";
                }
              }
              std::cout << "K[" << i <<"] = " << K[i] << std::endl;
            }
            it += 1;
          }

          std::cout << "After " << it << "iteration, ";
          std::cout << "modified Kp = " << K[0] << ", Ki = " << K[1] << ", Kd = " << K[2] << std::endl;
          pid.Init(K[0], K[1], K[2]);
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

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
