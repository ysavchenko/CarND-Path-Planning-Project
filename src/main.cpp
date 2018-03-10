#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

#define DISTANCE_SAFE_FRONT   15
#define DISTANCE_SAFE_BACK    5
#define POINT_INTERVAL        0.02
#define LANE_MIN    0
#define LANE_MAX    2
#define SPEED_MAX   49.5
#define SPEED_STEP  0.224
#define MPH_TO_MS   2.24
#define FUTURE_DISTANCE   50
#define TARGET_DISTANCE   30
#define POINTS_COUNT      50

// Class Lane encapsulates lane states (are they safe to change into and what is the safe speed there)
class Lane {

public:
  int index;          // Lane index (0, 1, 2)
  double interval;    // Planning interval (seconds)
  double d_from, d_to;  // Lane d coordinates (from, to)
  double car_s, car_speed;  // Our vehicle s and speed
  bool is_safe;
  double safe_speed;

  Lane(int index) {
    this->index = index;
    d_from = index * 4;
    d_to = (index + 1) * 4;
    is_safe = true;
  }

  // Returns the safe speed on this lane
  double getSafeSpeed() {
    return is_safe? safe_speed: 0;
  }

  // Returns the center of the lane
  double getCenter() {
    return (d_from + d_to) / 2;
  }

  // Inits the lane with current car position
  void setCarPosition(double s, double speed, double interval) {
    this->interval = interval;
    car_s = s;
    car_speed = speed;
    safe_speed = car_speed;   // By default we assume that current speed is safe
  }

  // Updates lane safety status with each vehicle on the road
  void updateStatus(vector<double> vehicle_info) {
    if (!is_safe)
      return;     // Lane already unsafe
    double d = vehicle_info[6];
    if (d < d_from || d > d_to)
      return;   // Vehicle is not in this lane

    double vx = vehicle_info[3];
    double vy = vehicle_info[4];
    double speed = sqrt(vx * vx + vy * vy);
    double s = vehicle_info[5];

    double distance_now = s - car_s;
    double distance_future = distance_now + (speed - car_speed) * interval;
    if (distance_now > DISTANCE_SAFE_FRONT && distance_future > DISTANCE_SAFE_FRONT)
      return; // The car is far away in front of our car, ignore it
    if (distance_now < -DISTANCE_SAFE_BACK && distance_future < -DISTANCE_SAFE_BACK)
      return; // The car is behind us, we can ignore it
    if (distance_now > 0) {
      double new_safe_speed = speed - (DISTANCE_SAFE_FRONT - distance_now) / interval;
      if (safe_speed > new_safe_speed)
        safe_speed = new_safe_speed;       // If car is in front of us -- adjust the safe speed
    } else {
      is_safe = false;
    }
  }
};

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  //start in lane 1(middle), valid values = {0,1,2}
	int lane = 1;
	double ref_vel = 0.0; //(mph) start at 0 and accelerate slowly
	int lane_change_cnt = 0; //keep track of the number of cycles since the last lane change

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane, &ref_vel, &lane_change_cnt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            int prev_size = previous_path_x.size();	
            double interval = prev_size * POINT_INTERVAL;

            double target_speed = min(ref_vel + SPEED_STEP, SPEED_MAX);

            vector<Lane> lanes;
            for (int index = LANE_MIN; index <= LANE_MAX; index++) {
              Lane lane_info = Lane(index);
              lane_info.setCarPosition(car_s, target_speed, interval);
              lanes.push_back(lane_info);
            }

            // Update lane info with vehicles
            for (int i = 0; i < sensor_fusion.size(); i++) {
              const vector<double>& vehicle_info = sensor_fusion[i];
              for (Lane& lane_info : lanes) {
                lane_info.updateStatus(vehicle_info);
              }
            }

            Lane& current_lane = lanes[lane];
            if (current_lane.safe_speed < target_speed) {
              // If current lane conditions prevent us from reaching target speed try to change the lane
              double speed_left = 0;
              double speed_right = 0;
              if (lane > LANE_MIN) {
                speed_left = lanes[lane - 1].getSafeSpeed();
              }
              if (lane < LANE_MAX) {
                speed_right = lanes[lane + 1].getSafeSpeed();
              }
              if (speed_left >= speed_right && speed_left > current_lane.safe_speed) {
                lane--;
              } else if (speed_right >= speed_left && speed_right > current_lane.safe_speed) {
                lane++;
              }
              //std::cout << "Lane speeds " << lanes[0].getSafeSpeed() << ", " << lanes[1].getSafeSpeed() << ", " << lanes[2].getSafeSpeed() << std::endl;
            }
            // Update target velocity using current lane's target speed
            current_lane = lanes[lane];
            if (ref_vel > current_lane.safe_speed)
              ref_vel = max(ref_vel - SPEED_STEP, current_lane.safe_speed);
            else
              ref_vel = min(ref_vel + SPEED_STEP, current_lane.safe_speed);
            
            // Prepare sparse path points to move spline through them later    
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if (prev_size < 2) {
              // Estimate previous position from current
              double prev_car_x = car_x - cos(ref_yaw);
              double prev_car_y = car_y - sin(ref_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            } else {
              // Use actual previous position
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
                
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);	
            }

            // Add our target points
            for (int future_step = 1; future_step <= 2; i++) {
              vector<double> next_point = getXY(car_s + future_step * FUTURE_DISTANCE, lanes[lane].getCenter(), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_point[0]);
              ptsy.push_back(next_point[1]);	
            }
			
            // Transform points into car coordinates
            for (int i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
              ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));				
            }

            // Create spline curve over the main points
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Re-use old path
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);				
            }

            double target_x = TARGET_DISTANCE;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            double x_add_on = 0;
	          // Use spline to fill out the rest of the POINTS_COUNT points
            for (int i = previous_path_x.size(); i < POINTS_COUNT; i++) {
              double N = (target_dist / (POINT_INTERVAL * ref_vel / MPH_TO_MS));
              double x_point = x_add_on + (target_x) / N;
              double y_point = s(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Transform coordinates from car to global system
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
