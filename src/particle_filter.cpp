/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Initialize all particles to first position (GPS-based estimations of x, y, theta with uncertainties) and all weights to 1. 
   * Random Gaussian noise to each particle.
   * http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   * http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  
  num_particles = 100;
  
  // Set GPS provided state
  double gps_x = x;
  double gps_y = y;
  double gps_theta = theta;
  
  // Add random Gaussian noise to each particle
  normal_distribution<double> noisy_x(gps_x, std[0]);
  normal_distribution<double> noisy_y(gps_y, std[1]);
  normal_distribution<double> noisy_theta(gps_theta, std[2]);
  
  // Generate particles
  for (int i=0; i<num_particles; i++){
    Particle particle;
    particle.id = i;
    particle.x = noisy_x(gen); // "gen" is the random engine initialized earlier
    particle.y = noisy_y(gen);
    particle.theta = noisy_theta(gen);
    particle.weight = 1.0;
    
    particles.push_back(particle);
    weights.push_back(particle.weight);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Predicts the state using motion model assuming a bicycle motion model.
   * For each particle the location is updated based on velocity and yaw rate.
   */
  
  std::default_random_engine gen;

  for (int i = 0; i < num_particles; i++)
  {
      Particle particle = particles[i];

      // If-Else block because there are two different methods of calculating position after motion depending on yaw rate. If it equals 0, then you can use known equations for constant velocity motion. 
      // Instead of a hard check of 0, adding a check for very low value of yaw_rate
      if (fabs(yaw_rate) < 0.0001) {
          particle.x = particle.x + velocity * delta_t * cos(particle.theta);
          particle.y = particle.y + velocity * delta_t * sin(particle.theta);
          particle.theta = particle.theta;
      }
      else {
          particle.x = particle.x + (velocity / yaw_rate) * (sin(particle.theta + (yaw_rate * delta_t)) - sin(particle.theta));
          particle.y = particle.y + (velocity / yaw_rate) * (cos(particle.theta) - cos(particle.theta + (yaw_rate * delta_t)));
          particle.theta = particle.theta + (yaw_rate * delta_t);
      }

      // Adding random Gaussian noise
      std::normal_distribution<double> noisy_x(particle.x, std_pos[0]);
      std::normal_distribution<double> noisy_y(particle.y, std_pos[1]);
      std::normal_distribution<double> noisy_theta(particle.theta, std_pos[2]);

      particles[i].x = noisy_x(gen);
      particles[i].y = noisy_y(gen);
      particles[i].theta = noisy_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Associate observations to landmarks.
   * NOTE: This method is useful as a helper during the updateWeights phase.
   */
  
  unsigned int numObservations = observations.size();
  unsigned int numPredictions = predicted.size();
  
  for (unsigned int i = 0; i< numObservations; i++){
    double minDistance = std::numeric_limits<double>::max();
    int index_map = -1;
    for (unsigned int j=0; j<numPredictions; j++){
      double x_distance = observations[i].x - predicted[j].x;
      double y_distance = observations[i].y - predicted[j].y;
      double distance = x_distance * x_distance + y_distance * y_distance;
      if (distance < minDistance){
        minDistance = distance;
        index_map =  predicted[j].id;
      }
    }
    observations[i].id = index_map;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks)
{
  /**
   * Updates the weights of each particle using a mult-variate Gaussian distribution.
   * Observations are given in the VEHICLE'S coordinate system. 
   * Particles are located according to the MAP'S coordinate system. 
   * We need to transform between the two systems. A homogenous transformation performs rotation and translation.
   */
  
  for (unsigned int i = 0; i < particles.size(); ++i)
    {
    Particle particle = particles[i];
    double prob = 1.0;

        for (unsigned int j = 0; j < observations.size(); j++)
        {
            // Homogenous Transformation (transforms observations from vehicle's coord. system to map's coordinate system)
            double x_m = particle.x + (cos(particle.theta) * observations[j].x) - (sin(particle.theta) * observations[j].y); // x_m >> transformed observation (TOBS) in x >> this means it is map coordinates
            double y_m = particle.y + (sin(particle.theta) * observations[j].x) + (cos(particle.theta) * observations[j].y); // y_m >> transformed observation (TOBS) in y >> this means it is map coordinates

            std::vector<Map::single_landmark_s> landmark_list = map_landmarks.landmark_list;
            double land_x; // x value of landmark
            double land_y; // y value of landmark
            double max_val = 2 * sensor_range;
            for (unsigned int k = 0; k < landmark_list.size(); k++)
            {
                // Calculate distance between particle and landmarks
                double local_land_x = landmark_list[k].x_f;
                double local_land_y = landmark_list[k].y_f;
                double distance = dist(x_m, y_m, local_land_x, local_land_y);
                if ((distance <= sensor_range) && (distance <= max_val))
                {
                  // Calculate multivariate Gaussian normal distribution
                  land_x = local_land_x;
                  land_y = local_land_y;
                  max_val = distance;
                  prob = multiv_prob(std_landmark[0], std_landmark[1], x_m, y_m, land_x, land_y);
                  particles[i].weight = prob;
                  weights[i] = prob;
                }
            }
        }
    }
}

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
    std::default_random_engine gen;
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::vector<Particle> resampled_particles;

    for (int n = 0; n < num_particles; ++n) 
    {
        Particle particle = particles[d(gen)];
        resampled_particles.push_back(particle);
    }
    particles = resampled_particles;

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
  particle.associations= associations;
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