#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include<stdio.h> 
#include<stdlib.h> 
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "MPC.h"
#include <fstream>



// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::ofstream;



int main() {
  uWS::Hub h;                    // Web server
  MPC mpc;                       // set MPC
  //Set variables for the csv file
  bool new_round = false;
  int rounds = 0;
  double init_x, init_y;
  bool set_init_xy = false;
  ofstream log;
  log.open("log.csv");


  h.onMessage([&mpc,&log,&set_init_xy,&rounds,&new_round](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
   
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') { 
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //Set state variables according to the given data.
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          //If car was near the 1st waypoint - increase rounds by 1.
          if(px >= -32.1617 - 1.5 && px <= -32.1617 +  1.5 && py >= 113.361 -  1.5 && py <= 113.361 +  1.5 && new_round ){
             rounds++;
             new_round=false;
           }
          // If we've to the second waypoint - set flag to false.
          if (px >= -43.4917 - 2 && px <= -43.4917 +  2 && py >= 105.941 -  2 && py <= 105.941 +  2)
             new_round = true;
           
           
          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          //Transform waypoints from the map coordinate system into car coordinate system.
          vector<double> waypoints_x;
          vector<double> waypoints_y;
          double dx, dy;
          
          for(int i = 0; i <ptsx.size(); i ++){
            dx = ptsx[i] - px;
            dy = ptsy[i] - py;
            
            waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
            waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
          }
          
          // Write to csv file car points / waypoints.
          log<<py<<","<<px<<","<<ptsy[0]<<","<<ptsx[0]<<"\n";
         
          double* ptrx = &waypoints_x[0];
          double* ptry = &waypoints_y[0];
          Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
          

          auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
          double cte = 0;
          double epsi = 0;

          // Send state values to MPC solver.
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          auto vars = mpc.Solve(state, coeffs);
          
          // Set actuators from MPC solution.
          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / mpc.MPC::deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;



      
          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
        
           
         

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
           
          

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          

       

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
         // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
       
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        log.close();
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h,&rounds,&log](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    log<<"rounds:"<<"\n";
    log<<rounds;
    log.close();
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
