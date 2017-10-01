#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <queue>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define LANE_CLEAR 0
#define PREP_CHANGE_LANE 1
#define FOLLOW 2
#define CHANGE_LANE 3

using namespace std;

// for convenience
using json = nlohmann::json;

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

class Trajectory {
public:
  Trajectory(const double x_car, const double y_car, const double yaw_car, const double s_car, const double drive_lane, vector<double> prev_path_x, vector<double> prev_path_y, const double speed_car, const int path_size);
  virtual ~Trajectory();
  //constexpr double pi() { return M_PI; }
  double deg2rad(double x);
  double rad2deg(double x);
  double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
  void startPoints();
  void makeSplinePts(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
  void getSpline();
  double solveSpline(const double x);
  void getTrajectoryPts(vector<double> &next_x_vals, vector<double> &next_y_vals, const double ref_vel, const bool too_close);
  
private:
  int prev_size;
  double max_vel;
  tk::spline f_spline;
  double car_x, car_y, car_yaw, car_s, lane, car_speed, global_x, global_y, global_yaw;
  vector<double> previous_path_x, previous_path_y, ptsx, ptsy;
};

Trajectory::Trajectory(const double x_car, const double y_car, const double yaw_car, const double s_car, const double drive_lane, vector<double> prev_path_x, vector<double> prev_path_y, const double speed_car, const int path_size)
{
  car_x = x_car;
  max_vel = 49.5;
  car_y = y_car;
  car_yaw = yaw_car;
  car_s = s_car;
  lane = drive_lane;
  previous_path_x = prev_path_x;
  previous_path_y = prev_path_y;
  car_speed = speed_car;
  prev_size = path_size;
}

Trajectory::~Trajectory() {
}

// For converting back and forth between radians and degrees.

double Trajectory::deg2rad(double x) { return x * M_PI / 180; }
double Trajectory::rad2deg(double x) { return x * 180 / M_PI; }

double Trajectory::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int Trajectory::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = Trajectory::distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int Trajectory::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = Trajectory::ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > M_PI/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Trajectory::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = Trajectory::NextWaypoint(x,y, theta, maps_x,maps_y);

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

	double frenet_d = Trajectory::distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = Trajectory::distance(center_x,center_y,x_x,x_y);
	double centerToRef = Trajectory::distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += Trajectory::distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += Trajectory::distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Trajectory::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

void Trajectory::startPoints(){
  // check if there are previous way points that can be used 
  
  if(prev_size < 2){
    global_x = car_x;
    global_y = car_y;
    global_yaw = Trajectory::deg2rad(car_yaw);

    // store the cars previous coordinates and transform them into the cars reference frame
    double shift_x = (global_x - cos(global_yaw)) - global_x;
    double shift_y = (global_y - sin(global_yaw)) - global_y;

    ptsx.push_back(shift_x*cos(0-global_yaw) - shift_y*sin(0-global_yaw));
    ptsy.push_back(shift_x*sin(0-global_yaw) + shift_y*cos(0-global_yaw));

    // store the cars current coordinates and transform them into the cars reference frame
    // since the current car coordinates are the origin store (0,0) 
    ptsx.push_back(0);
    ptsy.push_back(0);
  } else {
    global_x = previous_path_x[prev_size - 1];
    global_y = previous_path_y[prev_size - 1];
    double global_x_prev = previous_path_x[prev_size - 2];
    double global_y_prev = previous_path_y[prev_size - 2];

    global_yaw = atan2(global_y - global_y_prev, global_x - global_x_prev);

    // store the cars previous coordinates and transform them into the cars reference frame
    double shift_x = global_x_prev - global_x;
    double shift_y = global_y_prev - global_y;

    ptsx.push_back(shift_x*cos(0-global_yaw) - shift_y*sin(0-global_yaw));
    ptsy.push_back(shift_x*sin(0-global_yaw) + shift_y*cos(0-global_yaw));

    // store the cars current coordinates and transform them into the cars reference frame
    // since the current car coordinates are the origin store (0,0) 
    ptsx.push_back(0);
    ptsy.push_back(0);
  }
}

void Trajectory::makeSplinePts(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  // Frenet coordinates are referenced from the center yellow lines and positive d being on the right.
  double next_d = 2 + 4 * lane; //calculate the cars center d position based on the desired lane
  // get select way points in the future to predict a spline functions for the desired path  
  for(int i=1; i <= 3; i++){
    vector<double> xy_pts = Trajectory::getXY(car_s+(i*30), next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    // convert these future points into car centric coordinates
    double shift_x = xy_pts[0] - global_x;
    double shift_y = xy_pts[1] - global_y;

    ptsx.push_back(shift_x*cos(0-global_yaw) - shift_y*sin(0-global_yaw));
    ptsy.push_back(shift_x*sin(0-global_yaw) + shift_y*cos(0-global_yaw));
  }
}

void Trajectory::getSpline(){
  // create a function that intersects all points on the desired path
  f_spline.set_points(ptsx, ptsy);
}

double Trajectory::solveSpline(const double x){
  // create a function that intersects all points on the desired path
  
  return f_spline(x);
}

void Trajectory::getTrajectoryPts(vector<double> &next_x_vals, vector<double> &next_y_vals, const double ref_vel, const bool too_close) {

  double x_local = 0; // the current x point being considered
  double prev_x_local, prev_y_local;
  double acceleration = 0.003; //9 * 0.02; // max allowed acceleration per step
  int way_pts_tot = 50; // how many way points are to be predicted into the future 
  
  // Store the unused old way points to create a smooth path transition
  for(int i=0; i < prev_size; i++){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double step;
  int next_size = next_x_vals.size();
  // get the previous step between the last two previous trajectory points and set
  // that as the step size to prevent excess acceleration and jerk 
  if(next_size < 2){
    step = 0;
  } else {
    double step_x = next_x_vals[next_size - 1] - next_x_vals[next_size - 2];
    double step_y = next_y_vals[next_size - 1] - next_y_vals[next_size - 2];
    step = sqrt(step_x*step_x + step_y*step_y);
    
    prev_x_local = step_x;
    prev_y_local = step_y;
  }
  
  //cout << "Start   prev_x_local " << prev_x_local << "  prev_y_local " << prev_y_local << "  step " << step  << endl;

  const double mile_ph_to_meter_ps = 1609.344 / 3600.0; // 1Mph * 1609.344meter/h / 3600 = 0.44704 m/s
  const double max_step = max_vel * mile_ph_to_meter_ps * 0.02;
  double ref_step = std::min<double>(ref_vel * mile_ph_to_meter_ps * 0.02, max_step);

  // add new points onto the old way points upto 
  for(int i = 1; i < way_pts_tot - prev_size; i++){

    // check if we will potential have a collision and decelerate 
    if(too_close){
      step -= acceleration; // deceleration -5m/s
      if(step < ref_step){
        step = ref_step;
      }
    } else if(car_speed < ref_vel){
      step += acceleration;
      if(step > ref_step){
        step = ref_step;
      }
    } 

    
    x_local += step;
    double y_local = Trajectory::solveSpline(x_local);
    
    double diff_x = x_local - prev_x_local;
    double diff_y = y_local - prev_y_local;
    double diff_step = sqrt(diff_x*diff_x + diff_y*diff_y);
    int loop = 0;
    
    while(diff_step > max_step && loop < 7){
      double error = max_step/diff_step;
      
      x_local *= error;
      y_local = Trajectory::solveSpline(x_local);
    
      diff_x = x_local - prev_x_local;
      diff_y = y_local - prev_y_local;
      diff_step = sqrt(diff_x*diff_x + diff_y*diff_y);
      
      //cout << "loop " << loop << "  error " << error << "  diff_x " << diff_x << "  diff_y " << diff_y << "  diff_step " << diff_step << endl;
      loop++;
    }
    
    prev_x_local = x_local;
    prev_y_local = y_local;
    

    // convert the reference point from local car centric coordinates to global coordinates
    double x_point = x_local * cos(global_yaw) - y_local * sin(global_yaw);
    double y_point = x_local * sin(global_yaw) + y_local * cos(global_yaw);

    next_x_vals.push_back(x_point + global_x);
    next_y_vals.push_back(y_point + global_y);
  }
}

// Moving average class for a Queue 
// Original Source code by mehuld https://discuss.leetcode.com/topic/45731/c-solution-using-a-queue-calculate-average-in-o-1
class MovingAverage {
private:
    queue<int> q;
    int queue_size;
    int sum;
public:
  /** Initialize your data structure here. */
  MovingAverage() 
  {
     sum = 0;
  }

  void setSize(const int size)
  {
    queue_size = size;
  }
  
  int getSize()
  {
    return q.size();
  }
  
  void emptyQueue()
  {
    while (!q.empty())
    {
      q.pop();
    }
    sum = 0;
  }

  double nextAverage(const int val) 
  {
    if(q.size() < queue_size)
    {
      q.push(val);
      sum = sum + val;
    }
    else if(q.size() == queue_size)
    {
      int top = q.front();
      sum = sum - top;
      q.pop();
      q.push(val);
      sum = sum + val;
    }

    if(q.size() > 0)
      return double(sum)/(q.size());
    else
      return 0;
  }
};


class NextAction {
public:
  NextAction(const double max_speed);
  virtual ~NextAction();
  void setVehicleVariables(const double s_car, const double d_car, const double speed_car, const int path_size);
  int getAction(const vector<vector<double>> &sensor_fusion, double &ref_vel, bool &too_close, int &state);
  void prepLaneChangeState(const vector<vector<double>> &sensor_fusion, double &ref_vel);
private:
  double car_s, car_d, car_speed, max_vel, relative_vel_cost, max_vel_cost, s_cost;
  int lane, lane_left, lane_right, prev_size, next_state, filter_size;
  bool is_gap_left, is_gap_right, change_lane, is_new_state;//, too_close;
  double left_lane_cost, right_lane_cost, current_lane_cost, look_ahead_dist;
  
  MovingAverage left_ma, right_ma, current_ma;
};
  
NextAction::NextAction(const double max_speed)
{
  car_s = 0;
  car_d = 0;
  car_speed = 0;
  prev_size = 0;
  is_new_state = false;
  max_vel = max_speed;
  lane = -1;

  look_ahead_dist = 40.0;
  
  relative_vel_cost = 1; // cost off difference between this vehicle and the object
  max_vel_cost = 1; // cost between the speed limit and the object
  s_cost = 100;
  
  filter_size = 20;
  left_ma.setSize(filter_size);
  right_ma.setSize(filter_size);
  current_ma.setSize(filter_size);
}

NextAction::~NextAction() 
{
  
}

void NextAction::setVehicleVariables(const double s_car, const double d_car, const double speed_car, const int path_size)
{
  car_s = s_car;
  car_d = d_car;
  car_speed = speed_car;
  prev_size = path_size;

  if(lane < 0){
    lane = car_d / 4;
  }
  
  lane_left = std::max(lane-1, 0);
  lane_right = std::min(lane+1, 2);
  
  is_gap_left = true;
  is_gap_right = true;
  change_lane = false;
  //too_close = false;

  if(lane == 0){
    is_gap_left = false;
  } else if(lane == 2){
    is_gap_right = false;
  }
}

int NextAction::getAction(const vector<vector<double>> &sensor_fusion, double &ref_vel, bool &too_close, int &state)
{
  
  next_state = state;
  is_new_state = false;

  switch(state){
    case(LANE_CLEAR):
      ref_vel = max_vel;
      
      for(int i=0; i < sensor_fusion.size(); i++){
        // car is in my lane
        float d = sensor_fusion[i][6];
        double check_car_s = sensor_fusion[i][5];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double object_speed = sqrt(vx*vx + vy*vy);

        //check_car_s += prev_size * 0.02 * object_speed;

        // Check if there is a vehicle in front and in same lane 
        if((d < (2+4*lane+1.8)) && (d > (2+4*lane-1.8)) && (check_car_s > car_s) && (check_car_s - car_s < look_ahead_dist)){                
          // only change the next state if we haven't changed it already
          next_state = PREP_CHANGE_LANE;
          is_new_state = true;
          //cout << (1 - std::abs(car_s - check_car_s) / look_ahead_dist) << endl;
          break;
        } 
      }
      //cout << "LANE_CLEAR " << endl;
      break;
    case(PREP_CHANGE_LANE):
      left_lane_cost = 0;
      right_lane_cost = 0;
      current_lane_cost = 0;

      if(lane == 0){
        left_lane_cost = 10000;
      } else if(lane == 2){
        right_lane_cost = 10000;
      }
  
      NextAction::prepLaneChangeState(sensor_fusion, ref_vel);
      //cout << "PREP_CHANGE_LANE " << endl;
      break;
    case(FOLLOW):

      for(int i=0; i < sensor_fusion.size(); i++){
        // car is in my lane
        float d = sensor_fusion[i][6];
        double check_car_s = sensor_fusion[i][5];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double object_speed = sqrt(vx*vx + vy*vy);

        check_car_s += prev_size * 0.02 * object_speed;

        // only look at objects 15m behind and 40m in front that are in the 
        // lane to the left, right and same lane as travel
        //if((check_car_s > car_s-15) && (check_car_s - car_s < 40) && d < (2+4*lane_right+1.8) && d > (2+4*lane_left-1.8)){
        
        if((d < (2+4*lane+1.8)) && (d > (2+4*lane-1.8)) && (check_car_s > car_s) && (check_car_s - car_s < 25)){
          if(!is_new_state){
            next_state = PREP_CHANGE_LANE;
            is_new_state = true;
          }
          // slow down if the car in front is less than 20m ahead
          too_close = true;

          //stop the car if the object in front is stopped or we are very close to it
          if((object_speed <= 0.3) && (check_car_s - car_s < 5)){
            ref_vel = 0;
          }

          // TODO: add better maths to slow and follow the vehicle in front based of it's speed and distance
          if(check_car_s - car_s < 15){
            ref_vel = object_speed/2; // slow down to increase the distance between the car in front
          } else if(check_car_s - car_s < 10){
            ref_vel = object_speed/4; // slow a lot as we are too close to the car in front
          } else {
            ref_vel = object_speed; // slow down to the speed of the vehicle in front and follow it
          }
        } else if(!is_new_state){
          next_state = LANE_CLEAR;
        }
      }
      //cout << "FOLLOW" << endl;
      break;
    case(CHANGE_LANE):

      ref_vel = max_vel;
      
      if((car_d < (2+4*lane+0.5)) && (car_d > (2+4*lane-0.5))){
        next_state = LANE_CLEAR;
      }
      
      left_ma.emptyQueue();
      right_ma.emptyQueue();
      current_ma.emptyQueue();
      
      //cout << "CHANGE_LANE" << endl;
      break;
    default:
      next_state = LANE_CLEAR;
      //cout << "default" << endl;
  }
  
  state = next_state; // set the state for the next time around
  
  return lane;
}

void NextAction::prepLaneChangeState(const vector<vector<double>> &sensor_fusion, double &ref_vel)
{
  for(int i=0; i < sensor_fusion.size(); i++){
    // car is in my lane
    float d = sensor_fusion[i][6];
    double check_car_s = sensor_fusion[i][5];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double object_speed = sqrt(vx*vx + vy*vy);

    // only look at objects 15m behind and look_ahead_dist in front that are in the 
    // lane to the left, right and same lane as travel
    if((d > (2+4*lane_left-1.8)) && (d < (2+4*lane_right+1.8)) && (check_car_s > car_s-18) && (check_car_s - car_s < look_ahead_dist)){

      // velocity cost between this vehicle and the object speed
      double cost = std::abs(car_speed - object_speed) * relative_vel_cost;
      // velocity cost between the speed limit and the objects speed
      cost += std::abs(max_vel - object_speed) * max_vel_cost;
      // cost for the distance between this vehicle and the object 
      cost += (1 - std::abs(car_s - check_car_s) / look_ahead_dist) * s_cost;
      // cost for the distance between this vehicle and the object 1 second in the future
      cost += (std::abs((car_s + car_speed) - (check_car_s + object_speed)) / look_ahead_dist) * s_cost;
      
      // Check if there is a vehicle in front and in same lane 
      if((d < (2+4*lane+1.8)) && (d > (2+4*lane-1.8)) && (check_car_s > car_s)){   
        current_lane_cost += cost;

        if(check_car_s - car_s < 25){
          // only change the next state if a lane change has not yet to been set
          //if(next_state == PREP_CHANGE_LANE){
            next_state = FOLLOW;
          //}
          //break;
        } else if(!is_new_state){
          next_state = LANE_CLEAR;
        }
      } else if((d < (2+4*lane_left+2)) && (d > (2+4*lane_left-2))){
        // check if the object is in the left lane
        left_lane_cost += cost;
      } else if((d < (2+4*lane_right+2)) && (d > (2+4*lane_right-2))){
        // check if the object is in the right lane
        right_lane_cost += cost;
      } 
    }
  }
  
  left_lane_cost = left_ma.nextAverage(left_lane_cost);
  right_lane_cost = right_ma.nextAverage(right_lane_cost);
  current_lane_cost = current_ma.nextAverage(current_lane_cost);

  if(left_ma.getSize() == filter_size){
    if((left_lane_cost <= right_lane_cost) && (left_lane_cost < current_lane_cost)){
      // TODO: call change lane left state
      lane = std::max(lane - 1, 0);
      next_state = CHANGE_LANE;
    } else if((right_lane_cost < left_lane_cost) && (right_lane_cost < current_lane_cost)){
      // TODO: call change lane right state
      lane = std::min(lane + 1, 2);
      next_state = CHANGE_LANE;
    }
    //cout << "Lane " << lane << "   L " << left_lane_cost << "  C " << current_lane_cost << "  R " << right_lane_cost << endl;
  }
}




int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
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

  double ref_vel = 0;
  double max_vel = 49.5;
  int state = LANE_CLEAR;
  
  NextAction action(max_vel);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &action, &max_vel, &ref_vel, &state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            
            int prev_size = previous_path_x.size(); 
            if(prev_size > 20) prev_size = 20;

            if(prev_size > 0){
              car_s = end_path_s;
            }

            //ref_vel = 49.5;
            //int lane = car_d / 4;   
            bool too_close = false;
            
            action.setVehicleVariables(car_s, car_d, car_speed, prev_size);
            int lane = action.getAction(sensor_fusion, ref_vel, too_close, state);
            
            
            Trajectory car_tj(car_x, car_y, car_yaw, car_s, lane, previous_path_x, previous_path_y, car_speed, prev_size);
            car_tj.startPoints();

            car_tj.makeSplinePts(map_waypoints_s, map_waypoints_x, map_waypoints_y);
 
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
 
            // create a function that intersects all points on the desired path
            //tk::spline f_spline;
            car_tj.getSpline();
            //f_spline.set_points(ptsx, ptsy);

            car_tj.getTrajectoryPts(next_x_vals, next_y_vals, ref_vel, too_close);
            
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
