#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using std::string;
using std::vector;
using json_ = nlohmann::json;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Car's lane. Starting at middle lane.
  int lane = 1;

  // Reference velocity in mph.
  double ref_vel = 0.0; 

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json_::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          //checking previous path size
          if (previous_path_x.size()>0)
          {
              car_s = end_path_s;  //end path to avoid crash
          }


          //Analysing current host vehicle status (Situation Awarness for the Host Vehicle)
          bool car_at_right = false;
          bool car_at_left = false ;
          bool car_front_same_lane= false;


          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                float object_d = sensor_fusion[i][6];
                int car_inlane = -1;  //in lane car flag
                
                //checks to find if target car is in host lane or not
                if ( object_d > 0 && object_d < 4 ) {
                  car_inlane = 0;
                } else if ( object_d > 4 && object_d < 8 ) {
                  car_inlane = 1;
                } else if ( object_d > 8 && object_d < 12 ) {
                  car_inlane = 2;
                }
                if (car_inlane < 0) {
                  continue;
                }


                // Finding target vehicle speed 
                double velocity_x = sensor_fusion[i][3];
                double velocity_y = sensor_fusion[i][4];
                double object_speed = sqrt(velocity_y*velocity_y + velocity_x*velocity_x);
                double object_speed_check = sensor_fusion[i][5];

                // Update target vehile position using the computed speed.
                object_speed_check += ((double)previous_path_x.size()*0.02*object_speed);

                if (car_inlane == lane) // Car in the Host vehicle lane
                {                  
                  car_front_same_lane |= object_speed_check > car_s && object_speed_check - car_s < 30;
                } 
                else if (car_inlane - lane == -1) // Car in left lane to Host vehicle
                {                  
                  car_at_left |= car_s - 30 < object_speed_check && car_s + 30 > object_speed_check;
                } 
                else if (car_inlane - lane == 1) // Car in right lane to Host vehicle
                {                  
                  car_at_right |= car_s - 30 < object_speed_check && car_s + 30 > object_speed_check;
                }
            }

          double velocity_diff = 0;
            const double MAX_VEL = 45.5;
            const double MAX_ACCEL = .25;
            if (car_front_same_lane) // A target vehicle in front of Host vehicle
            { 
              if (!car_at_left && lane > 0) // if there is no car left and there is a left lane.
              {
                lane--; // Initiate left lane change.
              } 
              else if ( !car_at_right && lane != 2)// if there is no car left and there is a right lane.
              {
                lane++; // Initiate right lane change.
              } else 
              {
                velocity_diff -= MAX_ACCEL;
              }
            } 
            else 
            {
              if ( lane != 1 )  // If Host vehicle is not in the center lane
              { 
                if ((lane == 0 && !car_at_right) || (lane == 2 && !car_at_left)) 
                {
                  lane = 1; // Initiate host moving to center lane
                }
              }
              if ( ref_vel < MAX_VEL ) 
              {
                velocity_diff += MAX_ACCEL;
              }
            }


            //Trajectory Planning here
            vector<double> ptsx;
            vector<double> ptsy;

            double object_x = car_x;
            double object_y = car_y;
            double object_yaw = deg2rad(car_yaw);

            
            if (previous_path_x.size() < 2) //to check availability of previous points
            {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            } 
            else //if previous points available use them
            {
                object_x = previous_path_x[previous_path_x.size() - 1];
                object_y = previous_path_y[previous_path_x.size() - 1];

                double object_x_prev = previous_path_x[previous_path_x.size() - 2];
                double object_y_prev = previous_path_y[previous_path_x.size() - 2];
                object_yaw = atan2(object_y-object_y_prev, object_x-object_x_prev);

                ptsx.push_back(object_x_prev);
                ptsx.push_back(object_x);

                ptsy.push_back(object_y_prev);
                ptsy.push_back(object_y);
            }

            // Setting up target points in the future.
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Making coordinates to local car coordinates.
            for ( int i = 0; i < ptsx.size(); i++ ) 
            {
              double shift_x = ptsx[i] - object_x;
              double shift_y = ptsy[i] - object_y;

              ptsx[i] = shift_x * cos(0 - object_yaw) - shift_y * sin(0 - object_yaw);
              ptsy[i] = shift_x * sin(0 - object_yaw) + shift_y * cos(0 - object_yaw);
            }



            // Creating the spline.
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Output path points from previous path for continuity.
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            for ( int i = 0; i < previous_path_x.size(); i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }



            // Calculate distance y position on 35 m ahead.
            double target_x = 35.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for( int i = 1; i < 50 - previous_path_x.size(); i++ ) 
            {
              ref_vel += velocity_diff;
              if (ref_vel > MAX_VEL) 
              {
                ref_vel = MAX_VEL;
              } 
              else if (ref_vel < MAX_ACCEL) 
              {
                ref_vel = MAX_ACCEL;
              }
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(object_yaw) - y_ref * sin(object_yaw);
              y_point = x_ref * sin(object_yaw) + y_ref * cos(object_yaw);

              x_point += object_x;
              y_point += object_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
          



          json_ msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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