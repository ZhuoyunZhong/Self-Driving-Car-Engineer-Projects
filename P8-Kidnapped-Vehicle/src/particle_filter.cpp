/**
 * particle_filter.cpp
 *
 * Author: Joey Zhong
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

// Using random engine
std::default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  // Generalize noisy particles location
  std::normal_distribution<double> gen_x(x, std[0]);
  std::normal_distribution<double> gen_y(y, std[1]);
  std::normal_distribution<double> gen_theta(theta, std[2]);

  num_particles = 100;  // TODO: Set the number of particles

  // Initialize each filter
  for (int pi = 0; pi < num_particles; ++pi){
    Particle particle;
    particle.id = pi;
    particle.x = gen_x(gen);
    particle.y = gen_y(gen);
    particle.theta = gen_theta(gen);
    particle.weight = 1.0;

    particles.push_back(particle);
	weights.push_back(particle.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  // changes of x, y and delta
  double delta_x, delta_y, delta_theta;
  delta_theta = yaw_rate * delta_t;
  
  // Gaussian noise generator
  std::normal_distribution<double> gen_x(0, std_pos[0]);
  std::normal_distribution<double> gen_y(0, std_pos[1]);
  std::normal_distribution<double> gen_theta(0, std_pos[2]);

  // For yaw_rate != 0
  double delta_xy_factor = velocity/yaw_rate;

  // Update each filter
  for (auto& particle : particles){
    // previous theta
    double theta = particle.theta;

    // changes of x,y with Gaussian noise
    if (yaw_rate > 0){
      delta_x = delta_xy_factor * (sin(theta + yaw_rate*delta_t) - sin(theta));
      delta_y = delta_xy_factor * (cos(theta) - cos(theta + yaw_rate*delta_t));
    } else { // In case yaw_rate is 0
      delta_x = velocity * cos(theta) * delta_t;
      delta_y = velocity * sin(theta) * delta_t;
    }
    
    // update with noise
    particle.x += gen_x(gen) + delta_x;
    particle.y += gen_y(gen) + delta_y;
    particle.theta += gen_theta(gen) + delta_theta;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (auto& obs : observations){
    double min_dist = 9999;
    // Loop over all the landmarks to find the closet
    for (auto& pred : predicted){
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      // If closer
      if (distance < min_dist){
        min_dist = distance;
        obs.id = pred.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // For particle weight normalization
  double weight_sum = 0;

  // Update weight for each particle
  for (auto& particle : particles) {
    double p_x = particle.x;
    double p_y = particle.y;
    double p_theta = particle.theta;

    // Select in-range land marks
    vector<LandmarkObs> predicted;
    for (auto landmark : map_landmarks.landmark_list) {
      LandmarkObs inrange_landmark;
      if (dist(p_x, p_y, landmark.x_f, landmark.y_f) < sensor_range) {
        inrange_landmark.x = landmark.x_f;
        inrange_landmark.y = landmark.y_f;
        inrange_landmark.id = landmark.id_i;

        predicted.push_back(inrange_landmark);
      }
    }

    // Transform vehicle coordinate to map coordinate
    vector<LandmarkObs> trans_observations;
    trans_observations = transform_coord(observations, p_x, p_y, p_theta);
	
    // Associate observations with landmarks
    dataAssociation(predicted, trans_observations);

    // Set associations
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    for (auto obs : trans_observations) {
      associations.push_back(obs.id);
      sense_x.push_back(obs.x);
      sense_y.push_back(obs.y);
    }
    SetAssociations(particle, associations, sense_x, sense_y);
    
    // Calculate gauss weights
    double weight = 1.0;
	double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	double sig_x_2 = pow(std_landmark[0], 2);
	double sig_y_2 = pow(std_landmark[1], 2);
	
    for (auto obs : trans_observations) {
      // Observation
      double obs_x = obs.x;
      double obs_y = obs.y;
      // Associated landmark
      int id = obs.id;
	  
	  // Get x,y of the prediction associated with current observation  
      double mu_x = map_landmarks.landmark_list[id-1].x_f;
      double mu_y = map_landmarks.landmark_list[id-1].y_f;

      // for each observation
      double exponent = exp(-(pow(obs_x - mu_x, 2) / (2 * sig_x_2))
                            -(pow(obs_y - mu_y, 2) / (2 * sig_y_2)));
      double single_weight = gauss_norm * exponent;

	  // Update weight
      weight *= single_weight;
    }

    particle.weight = weight;
    weight_sum += weight;
  }

  // Normalize weights
  for (int i = 0; i < num_particles; ++i) {
    particles[i].weight /= weight_sum;
	weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  // Find the largest weight
  double max_weight = 0.0;
  for (auto& particle : particles){
    if (particle.weight > max_weight) {
      max_weight = particle.weight;
    }
  }
  double tt_maxw = 2 * max_weight;

  // Random number generator
  std::uniform_real_distribution<double> uni_r(0, tt_maxw);
  std::uniform_int_distribution<int> uni_i(0, num_particles-1);

  // Current index
  int index = uni_i(gen);
  // Increment
  double beta = 0;

  // Resampling Wheel Algorithm
  vector<Particle> re_particles;

  for (int i = 0; i < num_particles; ++i){
    beta += uni_r(gen);
    while (particles[index].weight < beta){
      beta -= particles[index].weight;
      index = (index + 1) % num_particles;
    }
    re_particles.push_back(particles[index]);
  }

  particles = re_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}