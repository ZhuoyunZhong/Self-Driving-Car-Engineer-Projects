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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  //Car starts in lane 1
  int lane = 1;
  
  // Define a reference velocity to target
  double ref_vel = 0.0; //mph
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];      
          
          int previous_size = previous_path_x.size(); 
          
          if (previous_size > 0 ) {
            car_s = end_path_s;
          }
          bool is_too_close = false;   
         
          // for changing lane
          bool front_left_lane = false;
          bool rare_left_lane = false;
          
          bool front_middle_lane = false;
          bool rare_middle_lane = false;                     
                    
          bool front_right_lane = false;
          bool rare_right_lane = false;             
          
          int car_ahead_distance = 30;
          int front_safety_distance = 35;
          int rare_safety_distance = 25;
          
          double car_speed_ahead = -1;
          double nearest_car_ahead_position = car_ahead_distance;
          
          // Detecting car ahead 
          for (int i = 0;  i < sensor_fusion.size(); i++){                       
            float d = sensor_fusion[i][6];                 
           
            //When car is in my lane
            if ((d < (2 + 4 * lane + 2)) && (d > (2 + 4 * lane - 2))){            
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt ( vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];     
              
              check_car_s += ((double)previous_size * .02 * check_speed);

              //car position and speed on the maneuvering time
              if ((check_car_s > car_s) && ((check_car_s - car_s) < car_ahead_distance )){          
                is_too_close = true; 
                
                if ( check_car_s < nearest_car_ahead_position){
                  nearest_car_ahead_position = check_car_s;
                  car_speed_ahead = check_speed;
                }
              }
            }
          }          
          
          
          // Lanes status
          for (int i = 0;  i < sensor_fusion.size(); i++){           
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt (vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            float d = sensor_fusion[i][6];            
            
            check_car_s += ((double)previous_size* .02 * check_speed); // if using previous points can project s value out
     
            // checking left lane gap
            if ((d < 4) && (check_car_s - car_s > front_safety_distance))
            {     
              // checking car at front.         
              if (check_speed > (car_speed*.44704*.8)){
                front_left_lane = true;
              }
            }
            else if ((d < 4) && (car_s - check_car_s > rare_safety_distance)){  
              // checking car at rear.
              if ((car_speed*.44704) > check_speed){
                rare_left_lane = true;
              }
            } 
            
            // checking middle lane
            if ((d > 4) && (d < 8) && (check_car_s - car_s > front_safety_distance)){      
              // checking car at front.
              if (check_speed > (car_speed*.44704*.8)){
                front_middle_lane = true;
              }
            }
            else if ((d > 4) && (d < 8) && (car_s - check_car_s > rare_safety_distance)){  
              // checking car at rear.
              if ((car_speed*.44704) > check_speed){
                rare_middle_lane = true;
              }
            }
            
            // checking right lane
            if ((d > 8) && (check_car_s - car_s > front_safety_distance)){    
              // checking car at front.
              if (check_speed > (car_speed*.44704*.8)){
                front_right_lane = true;
              }
            }
            else if ((d > 8) && (car_s - check_car_s > rare_safety_distance)){  
              // checking car at rear.
              if ((car_speed*.44704) > check_speed){
                rare_right_lane = true;
              }
            }         
          }
          
          // the availiability of changing lanes
          bool left_lane_gap_avail = front_left_lane && rare_left_lane; 
          bool middle_lane_gap_avail = front_middle_lane && rare_middle_lane; 
          bool right_lane_gap_avail = front_right_lane && rare_right_lane;                    
          

          // behavior planning
          if (is_too_close == true){
            ref_vel -= .224;
            // If the car is driving too slowly
            if (ref_vel < 40){
              // Car on the left lane
              if ((lane == 0) && (middle_lane_gap_avail)){
                lane++ ;
              }
              // Car on the middle lane
              else if (lane == 1){
                if (left_lane_gap_avail){
                  lane --;
                }
                else if (right_lane_gap_avail){
                  lane ++;
                }
              }              
              // Car on the right lane
              else if ((lane == 2) && (middle_lane_gap_avail)){
                lane--;
              }
            }
          }
          else if (ref_vel < 49.5){
            ref_vel += .224;               
          }

             
          //Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          vector<double> ptsx;
          vector<double> ptsy;
          
          //reference x, y and yaw states
          //either we will reference the starting point as where the car is or at the previous path end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //if previous size is almost empty, use the car as starting reference
          if (previous_size < 2){
            //Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
            //redefine reference state as previous path end point
            ref_x = previous_path_x[previous_size-1];
            ref_y = previous_path_y[previous_size-1];
            
            double ref_x_prev = previous_path_x[previous_size-2];
            double ref_y_prev = previous_path_y[previous_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
            
            // Use two points that make the path tangent to the previous Path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          //In frenet add evenly 30m spaced points ahead of the starting reference
          vector<double>next_wp0 = getXY(car_s + 30,(2 + 4 * lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double>next_wp1 = getXY(car_s + 60,(2 + 4 * lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);            
          vector<double>next_wp2 = getXY(car_s + 90,(2 + 4 * lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);  
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);          
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);     
          
          for (int i=0; i < ptsx.size(); i++){
            // shift car reference angle to 0 degrees
            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin (0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos (0 - ref_yaw));
          }
          
          //Using spline
          tk:: spline s;
          
          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);
          
          //Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);          
          }
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++){
            double N = (target_dist/ (.02 * ref_vel/2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos (ref_yaw) - y_ref * sin (ref_yaw));
            y_point = (x_ref * sin (ref_yaw) + y_ref * cos (ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);            
          }
       
          
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