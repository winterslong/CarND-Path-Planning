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
using json = nlohmann::json;

#define DBG_PRINT 0

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

#if DBG_PRINT
  cout << "Map file: " << map_file_ << endl;
  cout << "   map_waypoints_x.size()=" << map_waypoints_x.size() << endl;
  cout << "   map_waypoints_y.size()=" << map_waypoints_y.size() << endl;
  cout << "   map_waypoints_s.size()=" << map_waypoints_s.size() << endl;
  cout << "   map_waypoints_dx.size()=" << map_waypoints_dx.size() << endl;
  cout << "   map_waypoints_dy.size()=" << map_waypoints_dy.size() << endl;
#endif


  // Car's lane. Starting at middle lane.
  int lane = 1;

  // Reference velocity.
  double ref_vel = 0.0; // mph
  
  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,
                  &map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
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

          /* Sensor Fusion Data, a list of all other cars on the same side of the road.
             "sensor_fusion"-- A 2d vector of cars and then that car's 
             [0-car's unique ID, 
              1-car's x position in map coordinates, 
              2-car's y position in map coordinates, 
              3-car's x velocity in m/s, 
              4-car's y velocity in m/s, 
              5-car's s position in frenet coordinates, 
              6-car's d position in frenet coordinates. ]
            */          
          auto sensor_fusion = j[1]["sensor_fusion"];

          
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           
          // Provided previous path point size.
          int prev_size = previous_path_x.size();          
          // Preventing collitions.
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          // Prediction : Analysing other cars positions.
          // check flag
          bool car_in_front = false;
          bool car_in_left = false;
          bool car_in_right = false;
          
#if DBG_PRINT
          cout << "sensor_fusion.size=" << sensor_fusion.size() << endl;
#endif
          // iterate sensor fusion to update core flag
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            // current cat stats
            float d = sensor_fusion[i][6];
            
            int check_lane = -1;
            // is it on the same lane we are
            if ( d > 0 && d < 4 ) {
              check_lane = 0;
            } else if ( d > 4 && d < 8 ) {
              check_lane = 1;
            } else if ( d > 8 && d < 12 ) {
              check_lane = 2;
            }
#if DBG_PRINT
            cout << "   d=" << d << endl;
            cout << "   check_lane=" << check_lane << endl;
#endif
            if (check_lane < 0) {
              continue;
            }  
            
            // Find other car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            // Estimate car s position after executing previous trajectory.
            check_car_s += ((double)prev_size * 0.02 * check_car_speed);                               

            /* a car is considered "dangerous" when its distance to 
                  our car is less than 30 meters in front or behind us.
             */
            if ( check_lane == lane ) {
              // car_in_front check. Car in front of our lane
              car_in_front |= (check_car_s > car_s) && (check_car_s - car_s < 30);
            } else if ( check_lane - lane == -1 ) {
              // car_in_left check, Car left
              car_in_left |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
            } else if ( check_lane - lane == 1 ) {
              // car_in_right check, Car right
              car_in_right |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
            }              
#if DBG_PRINT
            cout << "   (vx, vy)=" << vx << ", " << vy << endl;
            cout << "   check_car_speed=" << check_car_speed << endl;
            cout << "   check_car_s=" << check_car_s << endl;
#endif          
          }
#if DBG_PRINT
          cout << "car_in_front=" << car_in_front << endl;
          cout << "car_in_left=" << car_in_left << endl;
          cout << "car_in_right=" << car_in_right << endl;
#endif
         
          /* A speed_diff is used for speed changes, this makes the car can change speed quickly 
           */
          double speed_diff = 0;
          const double MAX_SPEED = 49.5;
          const double MAX_ACC = 0.2; //.224;
          if (car_in_front) { 
            // If a car in front of us
            if ( !car_in_left && (lane > 0) ) {
              // if there is no car in left lane and there is a left lane. Then change lane left.
              lane--;
            } else if ( !car_in_right && (lane != 2) ){
              // if there is no car in right lane and there is a right lane. Then change lane right.
              lane++;
            } else {
              // Can not change lane, decrease the speed.              
              speed_diff -= MAX_ACC;
            }
          } else {
            // If there's no car in front of us, 
            // 1. check if we are not on the center lane.
            if ( lane != 1 ) { 
              if ( ((lane == 0) && !car_in_right ) || ((lane == 2) && !car_in_left ) ) {
                lane = 1; // Back to center lane
              }
            }

            // 2. No need to change lane, increases speed.
            if (ref_vel < MAX_SPEED) {
              speed_diff += MAX_ACC;
            }
          }
#if DBG_PRINT
          cout << "lane=" << lane << endl;
          cout << "speed_diff=" << speed_diff << endl;
#endif
          // add points 
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          /*the last two points of the previous trajectory (or the car position 
            if there are no previous trajectory) are used in conjunction
            three points at a far distance  to initialize the spline calculation. 
           */
          if ( prev_size < 2 ) {
              // There are not too many points
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
          
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
          
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          } else {
              /* Use the last two points that make the path tangent to the car
               */
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
          
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
          
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
          
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }
          
          // In Frenet: Setting up target points in the future.
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Making coordinates transformed (shift and rotation) to local car coordinates.
          for ( int i = 0; i < ptsx.size(); i++ ) {
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
          
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
#if DBG_PRINT
          cout << "ptsx.size=" << ptsx.size() << endl;
          cout << "ptsy.size=" << ptsy.size() << endl;
#endif
          
          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          /* In order to ensure more continuity on the trajectory,the pass
              trajectory points are copied to the new trajectory
           */
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate distance y position on 30 m ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          /*Fill up the rest of our path planner after filling it with previous points, 
            here we will always output 50 points
           */
          for( int i = 1; i < 50 - prev_size; i++ ) {
            ref_vel += speed_diff;
            if ( ref_vel > MAX_SPEED ) {
              ref_vel = MAX_SPEED;
            } else if ( ref_vel < MAX_ACC ) {
              ref_vel = MAX_ACC;
            }
            
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
          
            x_add_on = x_point;
          
            double x_ref = x_point;
            double y_ref = y_point;
          
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
          
            x_point += ref_x;
            y_point += ref_y;
          
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
#if DBG_PRINT
          cout << "next_x_vals.size=" << next_x_vals.size() << endl;
          cout << "next_y_vals.size=" << next_y_vals.size() << endl;
          cout << endl;
#endif
          
          json msgJson;
          
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
