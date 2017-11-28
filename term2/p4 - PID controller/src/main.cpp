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
   /*
   works but wiggly
   double init_Kp=-0.3;
   double init_Ki=0;
   double init_Kd=-1;


   works nice up to 30mph
   double init_Kp=-0.15;
   double init_Ki=0;
   double init_Kd=-1;


   works ok up to 60mph
   double init_Kp=-0.07;
   double init_Ki=0;
   double init_Kd=-1.2;

  better
  double init_Kp=-0.08;
  double init_Ki=0;
  double init_Kd=-3.0;

  nice
  double init_Kp=-0.06;
  double init_Ki=0;
  double init_Kd=-1.0;

  safe
  double init_Kp=-0.07;
  double init_Ki=0;
  double init_Kd=-1.0;


  double init_Kp=-0.06;
  double init_Ki=0;
  double init_Kd=-1.0;
  double init_throttle = 0.6;

 double throttle = 0.8 - (0.15 * cte)  - (0.15 * steer_value);


SMOOTH
  double init_Kp=-0.07;
  double init_Ki=0;
  double init_Kd=-1;
  double init_throttle = 0.6;

 double throttle = 0.9 - (0.15 * cte)  - (0.7 * fabs(steer_value)) - (0.005 * speed);

smooth faster

  double init_Kp=-0.07;
  double init_Ki=0;
  double init_Kd=-1;
  double init_throttle = 0.6;

  double throttle = 0.9 - (0.15 * cte)  - (0.8 * fabs(steer_value)) - (0.001 * speed);


even faster

  double init_Kp=-0.07;
  double init_Ki=0;
  double init_Kd=-1;
  double init_throttle = 0.6;

  double throttle = 0.9 - (0.15 * cte)  - (0.6 * fabs(steer_value)) - (0.001 * speed);


over 80mph (a little crazy)

  double init_Kp=-0.07;
  double init_Ki=0;
  double init_Kd=-1.1;
  double init_throttle = 0.6;

 double throttle = 1 - (0.11 * cte)  - (1.2 * fabs(steer_value)) - (0.001 * speed);



  double throttle = 0.9 - (0.11 * cte)  - (1.2 * fabs(steer_value)) - (0.001 * speed);


SLOW but "ok"

  double init_Kp=-0.085;
  double init_Ki=0;
  double init_Kd=-1.1;
  double init_throttle = 0.6;
  double init_speed_p = -0.1;
  double init_speed_d = -1;

50mph pretty smooth
double init_Kp=-0.12;
double init_Ki=0;
double init_Kd=-1.5;
double init_throttle = 0.6;
double init_speed_p = 0; //-0.12; //-0.2
double init_speed_d = 0; //1
double init_k_angle = 0;//-1.2;

double speed = 0.5 + speed_p * p_error + speed_d * d_error + fabs(steer_value) * K_angle;


70mph "ok"
double init_Kp=-0.12;
double init_Ki=0;
double init_Kd=-1.5;
double init_speed_p = -0.01; //-0.02; //-0.12; //-0.2
double init_speed_d = -3; //1
double init_k_angle = -0.01;//-0.02;//-1.2;
double init_throttle = 0.73;

based on ANGLE not steer value


  double init_Kp=0.13;
  double init_Ki=0;
  double init_Kd=1.5;

  // PID for speed
  double init_speed_p = 0.1; //-0.02; //-0.12; //-0.2
  double init_speed_d = 1; //1
    // throttle correction based on angle
  double init_k_angle = 0.01;//-0.02;//-1.2;
  double init_throttle = 0.5;

Submitted as safe
  //PID for steering
  double init_Kp=0.13;
  double init_Ki=0;
  double init_Kd=2;

  // PID for speed
  double init_speed_p = 0; //-0.02; //-0.12; //-0.2
  double init_speed_d = 0; //1
    // throttle correction based on angle
  double init_k_angle = 0;//-0.02;//-1.2;
  double init_throttle = 0.5;


  */

  //PID for steering
  double init_Kp=0.13;
  double init_Ki=0;
  double init_Kd=0;

  // PID for speed
  double init_speed_p = 0; //-0.02; //-0.12; //-0.2
  double init_speed_d = 0; //1
    // throttle correction based on angle
  double init_k_angle = 0;//-0.02;//-1.2;
  double init_throttle = 0.5;


  pid.Init(init_Kp, init_Ki, init_Kd, init_throttle, init_speed_p, init_speed_d, init_k_angle);

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
          if (cte > 5) {
            std::cout <<"you are out" << std::endl;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            getchar();
          }
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          std::cout << "angle "  << angle << std::endl;
          
          /*
          if ((fabs(angle) < 2) && (cte < 0.8)){
            pid.max_throttle =1;
          }
          else pid.max_throttle =-0.4;
          */


          steer_value = 0;
          std::cout << "speed: " << speed << std::endl;

          pid.UpdateError(cte);
          steer_value = pid.TotalError();


          /*
          double cte_diff = cte - prev_cte;
          steer_value = - (cte * Kp) - (cte_diff * Kd) - (Ki * total_cte);
          double prev_cte = cte;
          double total_cte = total_cte + cte;
           */

          // DEBUG
          double throttle = pid.SpeedError(angle);

          //double throttle =0.8;
          std::cout << "Steering Value: " << steer_value << std::endl;
          std::cout << "max throttle: " << pid.max_throttle << std::endl;
          std::cout << "throttle: " << throttle << std::endl;
          std::cout << "step " << pid.steps << std::endl << std::endl << std::endl;


          pid.steps += 1;
          pid.total_cte += fabs(cte);
          pid.speed_sum += speed;

          if (speed > pid.max_speed) {
            pid.max_speed = speed;
          }

          if (cte > pid.max_error){
            pid.max_error = cte;
          }

          pid.average_cte =  pid.total_cte / pid.steps;
          pid.average_speed =  pid.speed_sum / pid.steps;


          //(pid.steps % 1100) == 0)  - for each LAP

          // show stats after around 2 laps
          if ( pid.steps / 1000 == 2) {

            // 1100 if i change max_throttle 0.5-0.9
            std::cout << "\n LAP FINISHED" << std::endl  << std::endl  << std::endl;

            std::cout << "\n -------- STATS ----------" << std::endl;
            std::cout << "acumulated cte " << pid.total_cte << std::endl;
            std::cout << "average cte  " << pid.average_cte << std::endl;
            std::cout << "average speed " << pid.average_speed << std::endl;
            std::cout << "max cte  " << pid.max_error << std::endl;
            std::cout << "max speed " << pid.max_speed << std::endl;

            std::cout << "------------------------" << std::endl << std::endl;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            getchar();
          }



          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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
