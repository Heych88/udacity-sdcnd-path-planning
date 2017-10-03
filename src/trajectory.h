/*
 * trajectory.h
 * 
 * A simple trajectory path generator for autonomous vehicles using a spline.
 * 
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <fstream>
#include <math.h>
//#include <uWS/uWS.h>
//#include <chrono>
#include <iostream>
//#include <thread>
#include <vector>
#include <queue>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
//#include "json.hpp"
#include "spline.h"

using namespace std;

class Trajectory {
public:
  // class initialiser 
  Trajectory(const double x_car, const double y_car, const double yaw_car, 
          const double s_car, const double drive_lane, const vector<double> prev_path_x, 
          const vector<double> prev_path_y, const double speed_car, const int path_size);
  
  // destructor
  virtual ~Trajectory();    
  
  // Converts degrees to radians
  double deg2rad(double x);
  
  // Converts radians to degrees
  double rad2deg(double x);
  
  // Calculates the Euclidean distance between x and y coordinate points
  double distance(double x1, double y1, double x2, double y2);
  
  // Finds the closest map way point to the current vehicle position
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  
  // finds the next map way point  from the current way point
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  
  // Converts the map x & y points into Frenet coordinate points
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  
  // Converts the map Frenet coordinate points into x & y points
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
  
  // Gets the last trajectory points from the previous path or initialises them 
  // if there are no previous points to retrieve
  void startPoints();
  
  // Transform and convert the spline path into the vehicles reference frame 
  void makeSplinePts(const vector<double> map_waypoints_s, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y);
  
  // Create the spline that intersects all points on the desired path
  void getSpline();
  
  // Returns the splines y value at an x point 
  double solveSpline(const double x);
  
  // Creates the final trajectory path points
  void getTrajectoryPts(vector<double> &next_x_vals, vector<double> &next_y_vals, const double ref_vel, const bool too_close);
  
  // Checks and adjusts each point step to prevent speeding 
  void getStep(double &step, const double ref_vel, const bool too_close, double &x_local, double &y_local, double &prev_x_local, double &prev_y_local);
  
private:
  int prev_size; // size of the previous trajectory points 
  double max_vel; // max allowed vehicle velocity
  tk::spline f_spline; // spline maths function variable

  double car_x, car_y, car_yaw, car_s, car_speed; // current vehicle parameters
  
  // Car reference frame parameters for shifting between global and car reference frames
  double global_x, global_y, global_yaw;
  
  // lane number to travel in the future. 0 -> left, 1 -> middle, 2 -> right
  int lane; 
  
  vector<double> previous_path_x, previous_path_y; // previous trajectory (x,y) path points 
  vector<double> ptsx, ptsy; // new trajectory (x,y) path points 
};

/*
 * class initialiser
 * @param x_car, the vehicles current x coordinate value
 * @param y_car, the vehicles current y coordinate value
 * @param yaw_car, the vehicles current yaw angle
 * @param s_car, the vehicles current Frenet s coordinate value
 * @param drive_lane, the desired vehicle lane in which the car will be in.
 * @param prev_path_x, the vehicles previous trajectory path of unused x points
 * @param prev_path_y, the vehicles previous trajectory path of unused y points
 * @param speed_car, the vehicles current speed
 * @param path_size, the number of desired points in the previous path to reuse
 */
Trajectory::Trajectory(const double x_car, const double y_car, const double yaw_car, 
        const double s_car, const double drive_lane, const vector<double> prev_path_x, 
        const vector<double> prev_path_y, const double speed_car, const int path_size)
{
  car_x = x_car;
  max_vel = 49.0;
  car_y = y_car;
  car_yaw = yaw_car;
  car_s = s_car;
  lane = drive_lane;
  previous_path_x = prev_path_x;
  previous_path_y = prev_path_y;
  car_speed = speed_car;
  prev_size = path_size;
}

Trajectory::~Trajectory() 
{
    
}

// For converting back and forth between radians and degrees.
double Trajectory::deg2rad(double x) { return x * M_PI / 180; }
double Trajectory::rad2deg(double x) { return x * 180 / M_PI; }

/*
 * Calculates the Euclidean distance between x and y coordinate points
 * @param x1, coordinate point one x value 
 * @param y1, coordinate point one y value 
 * @param x2, coordinate point two x value 
 * @param y2, coordinate point two y value 
 * @return Euclidean distance
 */
double Trajectory::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/*
 * Finds the closest map way point to the current vehicle position
 * @param x, current vehicles coordinate x point 
 * @param y, current vehicles coordinate y point 
 * @param maps_x, global maps x points
 * @param maps_y, global maps y points
 * @return the index location of the nearest way point
 */
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

/*
 * finds the next map way point from the current (x, y) position
 * @param x, current vehicles coordinate x point 
 * @param y, current vehicles coordinate y point 
 * @param theta, current vehicles heading direction in radians
 * @param maps_x, global maps x points
 * @param maps_y, global maps y points
 * @return the index location of the nearest way point
 */
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

/*
 * Transform from Cartesian (x,y) coordinates to Frenet (s,d) coordinates
 * @param x, current vehicles coordinate x point 
 * @param y, current vehicles coordinate y point 
 * @param theta, current vehicles heading direction in radians
 * @param maps_x, global maps x points
 * @param maps_y, global maps y points
 * @return the vehicles (s,d) coordinates
 */
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

/*
 * Transform from Frenet (s,d) coordinates to Cartesian (x,y)
 * @param x, current vehicles coordinate x point 
 * @param y, current vehicles coordinate y point 
 * @param maps_s, global maps s points
 * @param maps_x, global maps x points
 * @param maps_y, global maps y points
 * @return the vehicles (s,d) coordinates
 */
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

/*
 * Gets the last trajectory points from the previous path or initialises them 
 * if there are no previous points to retrieve.
 */
void Trajectory::startPoints()
{
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

/*
 * Transform and convert the spline path into the vehicles reference frame
 * @param map_waypoints_s, global maps s points
 * @param map_waypoints_x, global maps x points
 * @param map_waypoints_y, global maps y points
 */
void Trajectory::makeSplinePts(const vector<double> map_waypoints_s, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y) 
{
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

/*
 *  Create the spline that intersects all points on the desired path
 */
void Trajectory::getSpline()
{
  // create a function that intersects all points on the desired path
  f_spline.set_points(ptsx, ptsy);
}

/*
 * Returns the splines y value at an x point 
 * @param x, x point to find the corresponding y value
 * @return corresponding y value
 */
double Trajectory::solveSpline(const double x)
{
  return f_spline(x);
}

/*
 * Creates the final trajectory path points
 * @param next_x_vals, vector to store the final paths x trajectory points
 * @param next_y_vals, vector to store the final paths y trajectory points
 * @param ref_vel, desired speed to be traveling in this directory
 * @param too_close, flag for being to close (true) to the vehicle in front.  
 */
void Trajectory::getTrajectoryPts(vector<double> &next_x_vals, vector<double> &next_y_vals, const double ref_vel, const bool too_close) 
{
  double x_local = 0; // the current x point being considered
  double y_local, prev_x_local, prev_y_local;
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

  // add new points onto the old way points upto 
  for(int i = 1; i < way_pts_tot - prev_size; i++){    
    
    // get the new (x,y) points and check that they are not causing speeding 
    Trajectory::getStep(step, ref_vel, too_close, x_local, y_local, prev_x_local, prev_y_local);

    // convert the reference point from local car reference frame to global coordinates
    double x_point = x_local * cos(global_yaw) - y_local * sin(global_yaw);
    double y_point = x_local * sin(global_yaw) + y_local * cos(global_yaw);

    next_x_vals.push_back(x_point + global_x); // store the new path points
    next_y_vals.push_back(y_point + global_y);
  }
}

/*
 * Calculates and adjusts each new (x, y) path point and prevents each step from speeding 
 * @param step, the previous points step value
 * @param ref_vel, desired speed to be traveling in this directory
 * @param too_close, flag for being to close (true) to the vehicle in front. 
 * @param x_local, new x point to check if it is speeding. This will be adjusted 
 *          to prevent speeding. 
 * @param y_local, new y point to check if it is speeding This will be adjusted 
 *          to prevent speeding. 
 * @param prev_x_local, previous x point to calculate the velocity step between
 * @param prev_y_local, previous y point to calculate the velocity step between 
 */
void Trajectory::getStep(double &step, const double ref_vel, const bool too_close, 
        double &x_local, double &y_local, double &prev_x_local, double &prev_y_local)
{
  double acceleration = 0.003; // max allowed acceleration per step 8m/s^2
  const double mile_ph_to_meter_ps = 1609.344 / 3600.0; // 1Mph * 1609.344meter/h / 3600 = 0.44704 m/s
  const double max_step = max_vel * mile_ph_to_meter_ps * 0.02;
  double ref_step = std::min<double>(ref_vel * mile_ph_to_meter_ps * 0.02, max_step);
  
  // check if we will potential have a collision and decelerate 
  if(too_close){
    step -= acceleration; // deceleration -8m/s
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
  y_local = Trajectory::solveSpline(x_local);
  
  // FIXME: fix velocity step check below as it causes excessive jerk on back part of track
  double diff_x = x_local - prev_x_local;
  double diff_y = y_local - prev_y_local;
  double diff_step = sqrt(diff_x*diff_x + diff_y*diff_y);
  int loop = 0;

  while(diff_step > max_step && loop < 5){
    double error = std::max(max_step/diff_step, 0.99);

    x_local *= error; // decrease the x_step by the error amount
    y_local = Trajectory::solveSpline(x_local);

    diff_x = x_local - prev_x_local;
    diff_y = y_local - prev_y_local;
    diff_step = sqrt(diff_x*diff_x + diff_y*diff_y);

    cout << "loop " << loop << "  error " << error << "  diff_x " << diff_x << "  diff_y " << diff_y << "  diff_step " << diff_step << endl;
    loop++;
  }

  prev_x_local = x_local; // update the previous step points
  prev_y_local = y_local;
}

#endif /* TRAJECTORY_H */

