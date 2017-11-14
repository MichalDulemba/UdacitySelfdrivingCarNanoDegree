/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <map>

#include "particle_filter.h"

using namespace std;
void ParticleFilter::showlandmarks(const Map &map_landmarks){

  std::cout << "landmark list size" << map_landmarks.landmark_list.size() << std::endl;
	for (int j=0; j < map_landmarks.landmark_list.size(); j++)
	{
	 std::cout << j << " - (" << map_landmarks.landmark_list[j].x_f << "," << map_landmarks.landmark_list[j].y_f << ") " << std::endl;
	}
   std:: cout << std::endl << std::endl;
}
void ParticleFilter::showparticles(){
    std::cout << std::endl << std::endl;
		for (int i=0; i< num_particles; i++){
			std::cout << "(" << particles[i].x << "," << particles[i].y << "." << particles[i].theta << "," << particles[i].weight << ") "  << std::endl;
		}
		std::cout << std::endl << std::endl;
		getchar();
}

void ParticleFilter::showobservations(std::vector<LandmarkObs> obs){
    std::cout << std::endl << std::endl;
		for (int i=0; i< obs.size(); i++){
			std::cout << "(" << obs[i].id << "," << obs[i].x << "," << obs[i].y << ") ";
		}
		std::cout << std::endl << std::endl;
		//getchar();
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
std::cout << "------------------------------ begin init -------------------------------------------" << std::endl;
num_particles = 100;
std:default_random_engine gen;
std::normal_distribution<double> N_x(x, std[0]);
std::normal_distribution<double> N_y(y, std[1]);
std::normal_distribution<double> N_theta(theta, std[2]);

for (int i=0; i< num_particles; i++){
	 Particle particle;
	 particle.id=i;
	 particle.x= N_x(gen);
	 particle.y= N_y(gen);
	 particle.theta = N_theta(gen);
	 particle.weight = 1;
	 particles.push_back(particle); // current set with all particles - vector
	 weights.push_back(particle.weight);  //insert 1
}
is_initialized = true;

showparticles();
std::cout << "------------------------------ /after init ------------------------------------------" << std::endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::cout << "------------------------------ begin prediction-----------------------------" << std::endl;
	getchar();

std::vector<Particle> temp_particles;
double x_f;
double y_f;
double yaw_f;

double x_0;
double y_0;
double init_yaw;
double weight;

default_random_engine gen;

for (int i=0; i<num_particles; i++){

    x_0 = particles[i].x;
		x_0 = particles[i].y;
		init_yaw = particles[i].theta;
		weight = particles[i].weight;
		std::cout << "weight" << weight << std::endl;

    Particle particle;

		if (yaw_rate == 0){

				x_f = x_0 + velocity*delta_t*cos(init_yaw);
				y_f = y_0 + velocity*delta_t*sin(init_yaw);
				yaw_f = init_yaw;
		}
		else{

				x_f = x_0 + velocity/yaw_rate * ( sin(init_yaw + (yaw_rate*delta_t)) - sin(init_yaw));
				y_f = y_0 + velocity/yaw_rate * ( cos(init_yaw) - cos(init_yaw+(yaw_rate*delta_t)) );
				yaw_f = init_yaw + yaw_rate*delta_t;
		}
    normal_distribution<double> N_x(x_f,std_pos[0]);
    normal_distribution<double> N_y(y_f,std_pos[1]);
		normal_distribution<double> N_theta(yaw_f,std_pos[2]);

    particle.id =i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = weight;
    temp_particles.push_back(particle) ;
}
particles = temp_particles;

std::cout << "------------------------------ after prediction ----------------------------------" << std::endl;
showparticles();
std::cout << "---------------------- ------- /after prediction ----------------------------------" << std::endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
double x_c;
double y_c;

double x_p;
double y_p;
double theta;

double x_m;
double y_m;

std::vector<LandmarkObs> translated_obs;
LandmarkObs temp_obs;
LandmarkObs before_trans;

int nearest_landmark;

std::cout << "--------------------------- begin updateWeights ------------------------------" << std::endl;


for (int p=0; p < num_particles; p++){

    std::cout << "\n ////////////////////////////-> ---- particle id = " << p << std::endl << std::endl;

		// TRANSLATE observations into global coordinates

    std::cout << "\n number of obs:" << observations.size() << std::endl;
		std::cout << "\n observations before trans:";
		showobservations(observations);

    translated_obs.clear();

		for (int i=0; i < observations.size(); i++){


		  x_c = observations[i].x;
			y_c = observations[i].y;

			 x_p = particles[p].x;
			 y_p = particles[p].y;
		   theta = particles[p].theta;

			x_m = x_p + (cos(theta) * x_c) - (sin(theta) * y_c);
			y_m = y_p + (sin(theta) * x_c) + (cos(theta) * y_c);

			temp_obs.x= x_m;
			temp_obs.y = y_m;
      temp_obs.id = i;

			translated_obs.push_back(temp_obs);

		}
    std::cout << "number of translated obs:" << translated_obs.size() << std::endl;
		std::cout << "\n observations after translation:";
    showobservations(translated_obs);

		particles[p].weight = 1;


		// FIND CLOSEST LANDMARK
    double dist_to_landmark;
		double a;
		double b;
		double A;
		double P;
		double P_final;
		double exponent;
		double obs_x;
		double obs_y;
		double l1_x;
		double l1_y;


    showlandmarks(map_landmarks);
    P_final = 1;

		for (int i=0; i< translated_obs.size();  i++){

			std::cout << "observation id= " << i << "- x:" << translated_obs[i].x << " y:" << translated_obs[i].y << endl;

			double max_distance = sensor_range;
			double prev_distance = max_distance;
		  nearest_landmark=0;
			dist_to_landmark = 0;

		  for (int j=0; j < map_landmarks.landmark_list.size(); j++)
			{
		  double landmark_x = map_landmarks.landmark_list[j].x_f;
			double landmark_y = map_landmarks.landmark_list[j].y_f;

				dist_to_landmark = sqrt(pow(translated_obs[i].x-landmark_x,2)  +pow(translated_obs[i].y-landmark_y,2));
			  if (dist_to_landmark < prev_distance){
		        nearest_landmark = j;
						prev_distance = dist_to_landmark;
				}
		  }

			std::cout << "closest landmark id= " << nearest_landmark << "- x:" << map_landmarks.landmark_list[nearest_landmark].x_f << " y:" << map_landmarks.landmark_list[nearest_landmark].y_f << endl;
			std::cout << "calc dist = " << dist_to_landmark << endl;

		// UPDATE WEIGHT


		//for (int i=0; i < translated_obs.size(); i++){


			a = 2*(std_landmark[0]*std_landmark[0]);
			b = 2*(std_landmark[1]*std_landmark[1]);

			obs_x = translated_obs[i].x;
			obs_y = translated_obs[i].y;

			l1_x = map_landmarks.landmark_list[nearest_landmark].x_f;
			l1_y = map_landmarks.landmark_list[nearest_landmark].y_f;

			A = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);

			exponent = ( (pow(obs_x-l1_x,2)/a) + (pow(obs_y-l1_y,2)/b) );

			P = A * exp(-exponent);
      std::cout << "partial P" << P << endl;

			P_final *= P;
      std::cout << "final P" << P_final << endl << endl;
		}
		//}
		particles[p].weight = P_final;

}

std::cout << "---------------------- after updateWeights ----------------------------- " << std::endl;
showparticles();
std::cout << "----------------------/ after updateWeights ---------------------------- " << std::endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// need to create weights vector
  std::cout << "\n --------------------------------begin resample ------------------------------ \n";

	std::vector <float> weights;

	for (int i=0; i<num_particles; i++){
		weights.push_back(particles[i].weight);
	}
  std::vector <Particle> particles_new;
  std::discrete_distribution<> d(weights.begin(), weights.end());


	std::random_device rd;
	std::mt19937 gen(rd());
	std::map<int, int> m;


	for (int n=0; n<num_particles; ++n) {
	       particles_new.push_back(particles[d(gen)]);
	}

	particles = particles_new;

	std::cout << "-------------------------------- end of resample --------------------" << std::endl;
	showparticles();
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
