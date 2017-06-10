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
#include <list>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // Set the number of particles.
  num_particles = 99;

  // Initialize random engine
  default_random_engine gen;

  // Create a normal (Gaussian) distribution
  normal_distribution<double> dist_x(x, std[0]); // TODO: test with 0 instead of x!
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    // Initialize all particles to first position (based on estimates of
    // x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    Particle parzival;
    parzival.id = i;
    parzival.weight = 1.0;
    parzival.x = dist_x(gen);
    parzival.y = dist_y(gen);
    parzival.theta = dist_theta(gen);

    // Add to set of particles
    particles.push_back(parzival);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // Initialize random engine
  default_random_engine gen;

  // Create a normal (Gaussian) distribution
  normal_distribution<double> dist_x(0, std_pos[0]); // TODO: test with 0 instead of particles[i].x!
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; ++i) {
    // Add measurements to each particle
    if (fabs(yaw_rate) > 0.00001){
      particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
    else {
      particles[i].x += velocity*cos(particles[i].theta)*delta_t;
      particles[i].y += velocity*sin(particles[i].theta)*delta_t;
      // theta stays the same in this case
    }
    // Add Gaussian noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  for (int t = 0; t < observations.size(); ++t){

    double dist_min = 99999;
    int nearest_neighbor = -1;

    // Take current observations and compare with...
    double x1 = observations[t].x;
    double y1 = observations[t].y;

    for (int i = 0; i < predicted.size(); ++i){
      // ... each prediction
      double x2 = predicted[i].x;
      double y2 = predicted[i].y;

      // Euclidean distance between transformed observation and landmark
      double dist_new = dist(x1, y1, x2, y2);
      if (dist_new < dist_min){
        // Associate landmark to transformed observation applying Nearest Neighbor concept
        nearest_neighbor = predicted[i].id;
        dist_min = dist_new;
      }
    }
    observations[t].id = nearest_neighbor;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

  vector<LandmarkObs> predicted;
  vector<LandmarkObs> tobservations = observations; // Vector for transformed observations

  // Update the weights of each particle
  for (int i = 0; i < num_particles; ++i) {

    // Get location of map landmarks within sensor range
    for (int p = 0; p < map_landmarks.landmark_list.size(); ++p){
      double range = dist(map_landmarks.landmark_list[p].x_f, map_landmarks.landmark_list[p].y_f, particles[i].x, particles[i].y);
      if (fabs(range) <= sensor_range){
        predicted.push_back({map_landmarks.landmark_list[p].id_i, map_landmarks.landmark_list[p].x_f, map_landmarks.landmark_list[p].y_f});
      }
    }

    // Convert observations from car coordinates into map coordinates in respect to the chosen particle
    for (int j = 0; j < observations.size(); ++j){
      tobservations[j].x = observations[j].x*cos(particles[i].theta)-observations[j].y*sin(particles[i].theta)+particles[i].x;
      tobservations[j].y = observations[j].x*sin(particles[i].theta)+observations[j].y*cos(particles[i].theta)+particles[i].y;
    }

    // Associate each transformed observation with a land mark identifier
    dataAssociation(predicted, tobservations);

    double final_weight = 1.0;

    // Calculated particles final weight as the product of each measurement's Multivariate-Gaussian probability
    for (int n = 0; n < tobservations.size(); ++n){
      int m;
      // Get the prediction associated with the current observation
      for (int l = 0; l < predicted.size(); ++l){
        if (predicted[l].id == tobservations[n].id){
          m = l;
        }
      }
      // Calculate the weight using Multivariate-Gaussian probability
      double q = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      double q_x = pow((tobservations[n].x - predicted[m].x),2.0) / (2 * pow(std_landmark[0], 2.0));
      double q_y = pow((tobservations[n].y - predicted[m].y),2.0) / (2 * pow(std_landmark[1], 2.0));
      // Particle's final weight is the product of of
      // each measurement's Multivariate-Gaussian probability
      final_weight *= q * exp(-(q_x + q_y));
    }
    particles[i].weight = final_weight;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Initialize random engine
  default_random_engine gen;

  // Initialization
  vector<double> weights;
  vector<Particle> particles_resampled;
  double max_weight = 0.0;
  // In order to create random value in range of amount of particles
  uniform_int_distribution<int> uni_dist_index(0, num_particles-1);

  // Get current weights as well as maximum weight
  for (int i = 0; i < num_particles; ++i) {
    weights.push_back(particles[i].weight);
    if (particles[i].weight > max_weight){
      max_weight = particles[i].weight;
    }
  }
  // In order to create random value in range of highest weight
  uniform_real_distribution<double> uni_dist_weight(0.0, max_weight);

  int index = uni_dist_index(gen);
  double beta = 0.0;

  // "Resampling Wheel" by Udacity
  for (int n = 0; n < num_particles; ++n){
    beta += uni_dist_weight(gen) * 2.0;
    while (beta > weights[index]){
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    particles_resampled.push_back(particles[index]);
  }
  particles = particles_resampled;
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
