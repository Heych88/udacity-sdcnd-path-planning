/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pathstates.h
 * Author: haidyn
 *
 * Created on 3 October 2017, 9:53 PM
 */

#ifndef PATHSTATES_H
#define PATHSTATES_H

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
#include "trajectory.h"
#include "filter.h" // MovingAverage class

#define LANE_CLEAR 0
#define PREP_CHANGE_LANE 1
#define FOLLOW 2
#define CHANGE_LEFT 3
#define CHANGE_RIGHT 4

using namespace std;

class NextAction {
public:
  NextAction(const double max_speed);
  virtual ~NextAction();
  void setVehicleVariables(const double s_car, const double d_car, const double speed_car, const int path_size);
  int getAction(const vector<vector<double>> &sensor_fusion, double &ref_vel, bool &too_close, int &state);
  void checkSurrounding(const vector<vector<double>> &sensor_fusion);
  int getFollowSpeed(bool &too_close);
private:
  double car_s, car_d, car_speed, max_vel, relative_vel_cost, max_vel_cost, s_cost;
  int lane, lane_left, lane_right, prev_size, next_state, filter_size;
  bool is_gap_left, is_gap_right, change_lane, is_new_state;//, too_close;
  double left_front_cost, right_front_cost, left_back_cost, right_back_cost, current_lane_cost; 
  double look_ahead_dist, action_ahead_dist, look_behind_dist, action_behind_dist, follow_dist;
  
  struct vehicle {
    double distance_s;
    double speed;   
  };
  
  vehicle center_front, left_front, right_front, left_back, right_back;
  
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

  look_ahead_dist = 60.0;
  action_ahead_dist = 50.0;
  look_behind_dist = 20.0;
  action_behind_dist = 10.0;
  follow_dist = 20.0;
  
  relative_vel_cost = 1; // cost off difference between this vehicle and the object
  max_vel_cost = 1; // cost between the speed limit and the object
  s_cost = 100;
  
  filter_size = 10;
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

  left_front_cost = 0;
  left_back_cost = 0;
  current_lane_cost = 0;
  right_front_cost = 0;
  right_back_cost = 0;
  

  if(lane == 0){
    is_gap_left = false;
    left_front_cost = 10000;
    left_back_cost = 10000;
  } else if(lane == 2){
    is_gap_right = false;
    right_front_cost = 10000;
    right_back_cost = 10000;
  } 
  
  // clear the vehicle struct values
  center_front.distance_s = 1000;
  center_front.speed = 1000;
  left_front.distance_s = 1000;
  left_front.speed = 1000;
  right_front.distance_s = 1000;
  right_front.speed = 1000;
  left_back.distance_s = -1000;
  left_back.speed = 1000;
  right_back.distance_s = -1000;
  right_back.speed = 1000;
}

void NextAction::checkSurrounding(const vector<vector<double>> &sensor_fusion)
{

  for(int i=0; i < sensor_fusion.size(); i++){
    // car is in my lane
    float d = sensor_fusion[i][6];
    double check_car_s = sensor_fusion[i][5];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double object_speed = sqrt(vx*vx + vy*vy);
    
    // add an offset to counter the sensor offset produced in the simulator which  
    // predicts the vehicles position 15m ahead of the actual vehicles position.
    double sensor_offset = 15.5;
    double distance_s = check_car_s - car_s + sensor_offset; 

    // only look at objects 15m behind and take action distance action_ahead_dist in front that are in the 
    // lane to the left, right and same lane as travel
    if((d > (2+4*lane_left-1.8)) && (d < (2+4*lane_right+1.8)) && (distance_s > -look_behind_dist) && (distance_s < look_ahead_dist)){ //(check_car_s > car_s-look_behind_dist)

      // velocity cost between this vehicle and the object speed
      double cost = std::abs(car_speed - object_speed) * relative_vel_cost;
      // velocity cost between the speed limit and the objects speed
      cost += std::abs(max_vel - object_speed) * max_vel_cost;
      // cost for the distance between this vehicle and the object 
      cost += (1 - std::abs(distance_s) / look_ahead_dist) * s_cost;
      // cost for the distance between this vehicle and the object 1 second in the future
      cost += (std::abs((check_car_s + object_speed) - (car_s + car_speed) + sensor_offset) / look_ahead_dist) * s_cost;
      
      // Check if there is a vehicle in front and in same lane 
      if((d < (2+4*lane+1.8)) && (d > (2+4*lane-1.8)) && (distance_s > 0)){  
        current_lane_cost += cost;
        
        if(center_front.distance_s > distance_s){
          center_front.distance_s = distance_s;
          center_front.speed = object_speed;
        }
      } else if((d < (2+4*lane_left+2)) && (d > (2+4*lane_left-2))){
        // check if the object is in the left lane
        if(distance_s > 0) {
          left_front_cost += cost;
          if(left_front.distance_s > distance_s){
            left_front.distance_s = distance_s;
            left_front.speed = object_speed;
          }
        } else {
          left_back_cost += cost;
          if(left_back.distance_s < distance_s){
            left_back.distance_s = distance_s;
            left_back.speed = object_speed;
          }
        }
        
        // check if there is a merge gap
        if((distance_s > -action_behind_dist) && (distance_s < follow_dist)){
          is_gap_left = false;
        }
      } else if((d < (2+4*lane_right+2)) && (d > (2+4*lane_right-2))){
        // check if the object is in the right lane
        if(distance_s > 0) {
          right_front_cost += cost;
          if(right_front.distance_s > distance_s){
            right_front.distance_s = distance_s;
            right_front.speed = object_speed;
          }
        } else {
          right_back_cost += cost;
          if(right_back.distance_s < distance_s){
            right_back.distance_s = distance_s;
            right_back.speed = object_speed;
          }
        }
        
        // check if there is a merge gap
        if((distance_s > -action_behind_dist) && (distance_s < follow_dist)){
          is_gap_right = false;
        }
      } 
    }
  }
}

int NextAction::getFollowSpeed(bool &too_close)
{
  // slow down if the car in front is less than 20m ahead
  too_close = true;
  
  double ref_vel;

  //stop the car if the object in front is stopped or we are very close to it
  if(((center_front.speed <= 0.3) && (center_front.distance_s < 5)) || (center_front.distance_s < 5)){
    ref_vel = 0;
  }

  // TODO: add better maths to slow and follow the vehicle in front based of it's speed and distance
  if(center_front.distance_s < 15){
    ref_vel = center_front.speed/2; // slow down to increase the distance between the car in front
  } else if(center_front.distance_s < 10){
    ref_vel = center_front.speed/4; // slow a lot as we are too close to the car in front
  } else {
    ref_vel = center_front.speed; // slow down to the speed of the vehicle in front and follow it
  }
  
  return ref_vel;
}

int NextAction::getAction(const vector<vector<double>> &sensor_fusion, double &ref_vel, bool &too_close, int &state)
{
  
  next_state = state;
  is_new_state = false;
  
  double left_cost, right_cost;

  switch(state){
    case(LANE_CLEAR):
      ref_vel = max_vel;

      NextAction::checkSurrounding(sensor_fusion);
      
      // Check if there is a vehicle in front and in same lane 
      if(center_front.distance_s < action_ahead_dist){               
        next_state = PREP_CHANGE_LANE;
      } 

      //cout << "LANE_CLEAR " << endl;
      break;
    case(PREP_CHANGE_LANE):
      NextAction::checkSurrounding(sensor_fusion);
      
      if(center_front.distance_s < follow_dist){
        next_state = FOLLOW;
      } else if(!is_new_state){
        next_state = LANE_CLEAR;
      }
      
      left_cost = left_ma.nextAverage(left_front_cost);
      right_cost = right_ma.nextAverage(right_front_cost);
      current_lane_cost = current_ma.nextAverage(current_lane_cost);

      if(left_ma.getSize() == filter_size){
        if(is_gap_left && (left_cost <= right_cost) && (left_cost < current_lane_cost)){
          // TODO: call change lane left state
          next_state = CHANGE_LEFT;
          lane = std::max(lane - 1, 0);
        } else if(is_gap_right && (right_cost < left_cost) && (right_cost < current_lane_cost)){
          // TODO: call change lane right state
          next_state = CHANGE_RIGHT;
          lane = std::min(lane + 1, 2);
        }
        //cout << "Lane " << lane << "   L " << left_cost << "  C " << current_lane_cost << "  R " << right_cost << endl;
      }
      
      //cout << "left: " << left_back.distance_s << "  center: " << center_front.distance_s << "  right: " << right_back.distance_s << endl;
      
      //cout << "PREP_CHANGE_LANE " << endl;
      break;
    case(FOLLOW):
      NextAction::checkSurrounding(sensor_fusion);
      
      if(center_front.distance_s < follow_dist) {
        next_state = PREP_CHANGE_LANE;

        ref_vel = NextAction::getFollowSpeed(too_close);
      } else {
        next_state = LANE_CLEAR;
      }
      //cout << "FOLLOW" << endl;
      break;
    case(CHANGE_LEFT):
      NextAction::checkSurrounding(sensor_fusion);
      
      if((center_front.distance_s < follow_dist) || (left_front.distance_s < follow_dist)) {
        ref_vel = NextAction::getFollowSpeed(too_close);
      } else {
        ref_vel = max_vel;
      }
      
      if((car_d < (2+4*lane+0.5)) && (car_d > (2+4*lane-0.5))){
        next_state = LANE_CLEAR;
      } 

      left_ma.emptyQueue();
      right_ma.emptyQueue();
      current_ma.emptyQueue();
      
      //cout << "CHANGE_LEFT" << endl;
      break;
    case(CHANGE_RIGHT):
      NextAction::checkSurrounding(sensor_fusion);
      
      if((center_front.distance_s < follow_dist) || (right_front.distance_s < follow_dist)) {
        ref_vel = NextAction::getFollowSpeed(too_close);
      } else {
        ref_vel = max_vel;
      }
        
      if((car_d < (2+4*lane+0.5)) && (car_d > (2+4*lane-0.5))){
        next_state = LANE_CLEAR;
      } 

      left_ma.emptyQueue();
      right_ma.emptyQueue();
      current_ma.emptyQueue();
      
      //cout << "CHANGE_RIGHT" << endl;
      break;
    default:
      next_state = LANE_CLEAR;
      //cout << "default" << endl;
  }
  
  state = next_state; // set the state for the next time around
  
  return lane;
}

#endif /* PATHSTATES_H */
