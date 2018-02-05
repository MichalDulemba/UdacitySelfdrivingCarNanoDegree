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
#include <chrono>
#include <thread>


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


// my helper functions

float last_car_curr_lane (vector <vector <float>> other_cars, int current_lane, vector <int> lane_middle, float car_s)
{
  int number_of_cars = other_cars.size();
  //std::cout << "number_of_cars" << number_of_cars;

  float smallest_distance=10000;
  float distance = 10000;
  int car_id = -1;

  for (int i=0; i<number_of_cars; i++){

      float d_pos_other_car = other_cars[i][6];
      int left_lane_edge = lane_middle[current_lane]-2;
      int right_lane_edge =lane_middle[current_lane]+2;

      if ( (d_pos_other_car > left_lane_edge) && (d_pos_other_car < right_lane_edge) )
      {
        double check_speed = other_cars[i][8];  //magnitude of velocity vector
        double check_car_s = other_cars[i][5];
        std::cout << "speed of checked car " << check_car_s;
        //check_car_s += ((double) prev_path_size*time_step*check_speed);
        //check_car_s = other_cars[i][5];
        distance = check_car_s - car_s;
        std::cout << " distance " << distance << " smallest dist " << smallest_distance << std::endl;
      }
      if ((distance > 0) && (distance < smallest_distance))
        {
          smallest_distance = distance;
          car_id = i;
        }
  }

  if (car_id != -1 ) {
    std::cout << "closest car is " << car_id << "on my lane at " << smallest_distance << std::endl;
    return car_id;
  }
  else {
    std::cout << "no cars" << std::endl;
    return -1;
  }

}


vector <int> check_for_empty_lanes(vector <vector <float>> other_cars, int lane_quantity, float car_s)
{
// which lanes are empty ahead of ego car
  std::cout << "my car at s " << car_s << std::endl;
  int number_of_cars = other_cars.size();
  int cars_in_front_by_lane[3] = {0,0,0};
  vector <int> empty_lanes;

  for (int i=0; i<lane_quantity; i++){
    // check for empty
    std::cout << "\n--------- testing for empty lane " << i << "---------" << std::endl;

    for (int j=0; j<number_of_cars; j++){

      int check_car_lane = other_cars[j][7];
      double check_car_s = other_cars[j][5];

     if (check_car_lane == i){

       std::cout << "    checking car id " << j << " on lane " << check_car_lane << " dist to ego car " <<  check_car_s - car_s << " check_car_s " <<  check_car_s << " car_s " << car_s << std::endl;
       if (((check_car_s - car_s) < 50)  && ((check_car_s - car_s) > -10))
       {
        //(check_car_s > car_s)  &&
        std::cout << "\n Lane " << i <<" blocked by car id=" << j  <<" \n ";
        cars_in_front_by_lane[check_car_lane]+=1;
       }
       std::cout << "\n";

     }


    }
    std::cout << "\n lane " << i << " cars " << cars_in_front_by_lane[i] << std::endl;
    if (cars_in_front_by_lane[i] == 0)
    {
      empty_lanes.push_back(i);
    }
  } //end of check all lanes


    if (empty_lanes.size() > 0 ) {
      std::cout<< "empty lanes  ";
      for (int i=0; i<empty_lanes.size();i++)
        {
          std::cout <<" " << empty_lanes[i];
        }
      }
      std::cout << "\n";
    return empty_lanes;

}


bool enough_space(vector <vector <float>> other_cars, int target_lane, float car_s)
{

  // later i should add checking for incoming cars
  bool man_possible=1;
  int number_of_cars = other_cars.size();

  for (int l=0; l<number_of_cars; l++){
        float other_car_pos = other_cars[l][5];

        if ( (other_cars[l][7] == target_lane) && (other_car_pos-car_s > -6) && (other_car_pos-car_s < 6)){
        man_possible=0;
      }
      else {
        man_possible=1;
      }

  }
  return man_possible;
}




int main() {
  uWS::Hub h;
  using namespace std::this_thread; // sleep_for, sleep_until
  using namespace std::chrono; // nanoseconds, system_clock, seconds
  sleep_for(seconds(8));

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

  double starting_velocity = 0.0;
  double target_velocity = starting_velocity; // iniate with 0 to avoid jerk at the start
  double previous_target_velocity = 0.0;
  int current_lane =1; // ### FIX SCOPE ###

  h.onMessage([&target_velocity, &previous_target_velocity, &current_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            previous_target_velocity = target_velocity;

          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

		        std::cout << "carx " << car_x << "cary " << car_y << std::endl;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            std::cout <<"number of visible cars " << j[1]["sensor_fusion"].size() << std::endl;

            vector <vector <float>> other_cars;

            for (int i=0; i<j[1]["sensor_fusion"].size(); i++){

              vector  <float> single_car = j[1]["sensor_fusion"][i];
              float other_car_lane = j[1]["sensor_fusion"][i][6];
              //std::cout << "d" << other_car_lane <<std::endl;
              if (floor(other_car_lane / 4) > 2 || floor(other_car_lane / 4) <0)
                other_car_lane = -1;
              else
                other_car_lane = floor(other_car_lane / 4);
              //std::cout << "d/4  = " << other_car_lane <<std::endl;
              // what if changing lane
              single_car.push_back(other_car_lane);

              double vx = j[1]["sensor_fusion"][i][3];
              double vy = j[1]["sensor_fusion"][i][4];
              double speed_magnitude = sqrt(vx*vx+vy*vy);
              single_car.push_back(speed_magnitude);

              double check_car_s = j[1]["sensor_fusion"][i][5];
              float distance_to_ego_car = check_car_s - car_s;
              std::cout << "    checking car id " << i << " on lane " << other_car_lane << " dist to ego car " <<  distance_to_ego_car << " check_car_s " <<  check_car_s << " car_s " << car_s << std::endl;

              single_car.push_back(distance_to_ego_car);

              //show car data
               for(int i=0; i<single_car.size(); ++i)
                   std::cout << single_car[i] << ' ';
               std::cout << std::endl;

              other_cars.push_back(single_car);
            }


            //auto sensor_fusion = j[1]["sensor_fusion"];


          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


            double time_step = 0.02; // seconds
            double distance_inc = 0.5;
            int lane_width = 4;
            int trajectory_points_number = 50;
            int lane_quantity = 3;
            vector<double> spline_x_vals;
            vector<double> spline_y_vals;
            int smooth_point_dist = 40;

            // create middle for each lane
            vector <int> lane_middle;
            for (int i=0; i<lane_quantity; i++){
              lane_middle.push_back(2+i*lane_width);
            }
            //std:: cout << "############# Start ###########" << "\n";
            std:: cout << "current_lane " << current_lane << " lane middle " << lane_middle[current_lane]  <<  std::endl;

            double ref_car_x = car_x;
            double ref_car_y = car_y;
            double ref_car_yaw = deg2rad(car_yaw);
            int prev_path_size = previous_path_x.size();
            int number_of_cars = other_cars.size();
            int minimum_distance = 20;
            int visible_distance = 100;

            bool too_close = false;

            int cars_in_front_by_lane[3] = {0,0,0};

            std::vector<int> potential_lanes;

            // check for other possible lanes
            if ((current_lane==0) || (current_lane==2))
                potential_lanes.push_back(1);
            else {
                potential_lanes.push_back(0);
                potential_lanes.push_back(2);
            }


            std::cout << "\n potential lanes : ";
            for (int i=0; i<potential_lanes.size(); i++){
                  // check for empty
                   std :: cout <<" "<< potential_lanes[i] << " ";
            }
            std::cout << "\n";


            int last = last_car_curr_lane(other_cars, current_lane, lane_middle, car_s);
            float dist_to_last;
            float last_car_speed;

            if (last!=-1){
            //check_for_empty_lanes(other_cars, lane_quantity, car_s);
            dist_to_last = other_cars[last][5] - car_s;
            last_car_speed = other_cars[last][8];
            }
            else{
              dist_to_last=0;
              last_car_speed=0;
            }
            int target_lane=-1;

            // long dist check


            if ( (dist_to_last > 0) && (dist_to_last > minimum_distance) && (dist_to_last < visible_distance)){
                cout << "Long range check" << std::endl;
                cout << "car ahead at dist " << dist_to_last  << std::endl;
                vector <int> empty_lanes =  check_for_empty_lanes(other_cars, lane_quantity, car_s);

                for (int k=0; k<potential_lanes.size(); k++){
                    if (std::find(empty_lanes.begin(), empty_lanes.end(), potential_lanes[k]) !=empty_lanes.end() )
                      {
                        std::cout << "found empty potential lane " << potential_lanes[k] << "\n";
                        target_lane = potential_lanes[k];
                        smooth_point_dist = 60;
                        break;
                      }
                    else
                      {
                        cout<< "potential lane" << potential_lanes[k] << " is FULL \n";
                        target_lane == current_lane;
                      }
                }
            }


            // short dist check
            else if ( (dist_to_last > 0) && (dist_to_last < minimum_distance )) {
                    //target_velocity = 29.5;
                cout << "Close range check" << std::endl;
                std::cout << "too close \n";
                too_close = true;

                vector <int> empty_lanes =  check_for_empty_lanes(other_cars, lane_quantity, car_s);

                for (int k=0; k<potential_lanes.size(); k++){


                    if (std::find(empty_lanes.begin(), empty_lanes.end(), potential_lanes[k]) !=empty_lanes.end() )
                      {
                        std::cout << "--found empty potential lane " << potential_lanes[k] << "\n";
                        target_lane = potential_lanes[k];
                        smooth_point_dist = 40;
                        break;
                      }
                    else
                      {
                        cout<< "--potential lane" << potential_lanes[k] << " is FULL \n";
                        target_lane == current_lane;
                      }
                }
              }


            if ( (current_lane!=target_lane) && (target_lane!=-1) && enough_space(other_cars, target_lane, car_s) )
              {
                current_lane = target_lane;
                std:: cout << "Changing LANE to " << target_lane << std::endl;
              }

            else{
                current_lane=current_lane;
            }


            std::cout << " Speed control \n\n" ;
            std::cout << "current vel " << car_speed << " target_velocity " << target_velocity << std::endl;

            if (too_close==true)
                {

                  // small PID controller for speedCTE = robot.y

                  // float diff_SPEED = car_speed - previous_target_velocity;
                  // float speed_error = target_velocity - last_car_speed;
                  // std:: cout << "last car speed " << last_car_speed << " diff speed " << diff_SPEED << " speed_error " << speed_error << std::endl;
                  // float tau_p = 0.11;
                  // float tau_d = 1.5;
                  //
                  // float PID_speed = tau_p * speed_error + tau_d * diff_SPEED;
                  // std:: cout << "pid speed " << PID_speed << std::endl;
                  //
                  //
                  // target_velocity = target_velocity - PID_speed;
                  target_velocity -= 0.448;
                }

            else if ( (too_close==false) && (target_velocity < 49.5) ){
              std:: cout << "Speed up" << std::endl;
              target_velocity += 0.448; //448 is fine
            }


            std::cout << " prev target velocity " << previous_target_velocity << std::endl;
            std::cout << "target velocity " << target_velocity << std::endl;

            // std:: cout << "############# End ################" << "\n";



            /// ------ begin of spline points creation

            // "continuity points"

            if (prev_path_size >0)
            {
              car_s = end_path_s;
            }

            if ( (prev_path_size < 2) )  //  || (too_close==true)dont use old point when changing speed
            {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                spline_x_vals.push_back(prev_car_x);
                spline_x_vals.push_back(car_x);

                spline_y_vals.push_back(prev_car_y);
                spline_y_vals.push_back(car_y);

            }
            else
            {

              ref_car_x = previous_path_x[prev_path_size-1];
              ref_car_y = previous_path_y[prev_path_size-1];

              double prev_next_last_x= previous_path_x[prev_path_size-2];
              double prev_next_last_y= previous_path_y[prev_path_size-2];

              double at_x = ref_car_x-prev_next_last_x;
              double at_y = ref_car_y-prev_next_last_y;

              ref_car_yaw = atan2(at_y, at_x);

              spline_x_vals.push_back(prev_next_last_x);
              spline_x_vals.push_back(ref_car_x);

              spline_y_vals.push_back(prev_next_last_y);
              spline_y_vals.push_back(ref_car_y);


            }
            // new points based on the chosen lane

            //std::cout << "spline x size " << spline_x_vals.size() <<  "spline y size " << spline_y_vals.size() << std::endl;
            //std::cout << "current line "<< current_lane << "lane middle " << lane_middle[current_lane];
            vector <double> next_wp0 = getXY(car_s+smooth_point_dist, lane_middle[current_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector <double> next_wp1 = getXY(car_s+(smooth_point_dist*2), lane_middle[current_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector <double> next_wp2 = getXY(car_s+(smooth_point_dist*3), lane_middle[current_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);

            spline_x_vals.push_back(next_wp0[0]);
            spline_x_vals.push_back(next_wp1[0]);
            spline_x_vals.push_back(next_wp2[0]);

            spline_y_vals.push_back(next_wp0[1]);
            spline_y_vals.push_back(next_wp1[1]);
            spline_y_vals.push_back(next_wp2[1]);

            //std::cout << "spline x size " << spline_x_vals.size() <<  "spline y size " << spline_y_vals.size() << std::endl;


            // show points before transform
            // for (int i=0; i<spline_x_vals.size(); i++){
            //   std::cout << "xy " << spline_x_vals[i] << " " << spline_y_vals[i] << std::endl;
            // }
            /// ------ end of spline points creation

            std::cout << std::endl;

            for (int i=0; i<spline_x_vals.size(); i++)
            {
              // transform to local coordinates

              double shift_x = spline_x_vals[i]-ref_car_x;
              double shift_y = spline_y_vals[i]-ref_car_y;

              spline_x_vals[i] = (shift_x * cos(0-ref_car_yaw)-shift_y*sin(0-ref_car_yaw));
              spline_y_vals[i] = (shift_x * sin(0-ref_car_yaw)+shift_y*cos(0-ref_car_yaw));
            }
            // show points after tranform
            // for (int i=0; i<spline_x_vals.size(); i++){
            //   std::cout << "xy " << spline_x_vals[i] << " " << spline_y_vals[i] << std::endl;
            // }

            std::cout << std::endl;

            tk::spline s;
            s.set_points(spline_x_vals, spline_y_vals);

            std::cout << "spline calculated correctly " << std::endl;

            // copy old points to the final points for continuity
            std:: cout << "prev path size " << prev_path_size << std::endl;

            for (int i=0; i<prev_path_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }


            double target_x = 30;
            double target_y = s(target_x);  // how far is the x=30 point
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            std::cout << "target x " << target_x << " target y " << target_y << " target dist " << target_dist << std::endl << std::endl << std::endl;

            double x_add_on = 0;


            for (int i=1; i < trajectory_points_number-prev_path_size; i++){
              double converted_speed =  target_velocity/2.24;

              double N = (target_dist/(time_step*converted_speed)); // 2.24 is conversion to m/s from mph

              double x_point = x_add_on+(target_x)/N; // 0 + target_x divided into N parts
              double y_point = s(x_point);

              x_add_on = x_point;

              // convert from local to global

              double global_x_ref = x_point;
              double global_y_ref = y_point;

              x_point = (global_x_ref * cos(ref_car_yaw) - global_y_ref *sin(ref_car_yaw));
              y_point = (global_x_ref * sin(ref_car_yaw) + global_y_ref *cos(ref_car_yaw));


              x_point += ref_car_x;
              y_point += ref_car_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;
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
