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

#include "particle_filter.h"

using namespace std;

LandmarkObs transformCoordinate(LandmarkObs obs, Particle particle) {
	double radian_obs_in_car = atan2(obs.y, obs.x);
	// cout << radian_obs_in_car << endl;
	double radian_obs_in_map = radian_obs_in_car + particle.theta;
	// cout << radian_obs_in_map << endl;
	double distance = sqrt(obs.x * obs.x + obs.y * obs.y);
	// cout << distance << endl;
	LandmarkObs transformed;
	transformed.id = obs.id;
	transformed.x = particle.x + distance * cos(radian_obs_in_map);
	transformed.y = particle.y + distance * sin(radian_obs_in_map);
	return transformed;
}

void associate(std::vector<LandmarkObs> observations, Map map_landmarks, Particle& p) {
	std::vector<int> associated;
	p.associations.clear();
	p.sense_x.clear();
	p.sense_y.clear();
	for (int j = 0; j < observations.size(); ++j)
	{
		LandmarkObs o = observations[j];
		std::vector<double> dists;
		for (int i = 0; i < map_landmarks.landmark_list.size(); ++i)
		{
			Map::single_landmark_s lm = map_landmarks.landmark_list[i];
			dists.push_back(sqrt((o.x-lm.x_f)*(o.x-lm.x_f) + (o.y-lm.y_f)*(o.y-lm.y_f)));
		}
		std::vector<double>::iterator min_result;
		min_result = min_element(dists.begin(), dists.end());
		int index = distance(dists.begin(), min_result);
		p.associations.push_back(index);
		p.sense_x.push_back(map_landmarks.landmark_list[index].x_f);
		p.sense_y.push_back(map_landmarks.landmark_list[index].y_f);
	}
}

void calculateWeight(std::vector<LandmarkObs> observations, Map map_landmarks, Particle& particle, double std_landmark[]) {
	double ox, oy, lx, ly, stdx, stdy;
	std::vector<double> weights;
	for (int i = 0; i < particle.associations.size(); ++i)
	{
		int landmark_index = particle.associations[i];
		ox = observations[i].x;
		oy = observations[i].y;
		stdx = std_landmark[0];
		stdy = std_landmark[1];
		lx = particle.sense_x[i];
		ly = particle.sense_y[i];
		double weight;
		weight = exp(-0.5*((ox-lx)*(ox-lx)/(stdx*stdx) + (oy-ly)*(oy-ly)/(stdy*stdy))) / (2*M_PI*stdx*stdy);
		weights.push_back(weight);
	}
	double producted_wight = 1;
	for (int i = 0; i < weights.size(); ++i)
	{
		producted_wight *= weights[i];
	}
	particle.weight = producted_wight;
}

void normalizeWeights(std::vector<Particle>& particles) {
	double weight_sum = 0;
	for (int i = 0; i < particles.size(); ++i)
	{
		weight_sum += particles[i].weight;
	}
	for (int i = 0; i < particles.size(); ++i)
	{
		particles[i].weight = particles[i].weight / weight_sum;
	}
}

// -------------------------- requrird methods -----------------------
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Sets the number of particles. Initializes all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Adds random Gaussian noise to each particle.
  
  // Set number of particles
  // *** Can be tuned ***
  num_particles = 100;
    
  // Resize weights vector based on num_particles
  weights.resize(num_particles);
    
  // Resize vector of particles
  particles.resize(num_particles);
  
  // Engine for later generation of particles
  random_device rd;
  default_random_engine gen(rd());
    
  // Creates a normal (Gaussian) distribution for x, y and theta (yaw).
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
    
  // Initializes particles - from the normal distributions set above
  for (int i = 0; i < num_particles; ++i) {
      
    // Add generated particle data to particles class
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
      
  }
    
  // Show as initialized; no need for prediction yet
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Adds measurements for velocity and yaw_rate to each particle and adds random Gaussian noise.
  
  // Engine for later generation of particles
  default_random_engine gen;
  
  // Make distributions for adding noise
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  // Different equations based on if yaw_rate is zero or not
  for (int i = 0; i < num_particles; ++i) {
    
    if (abs(yaw_rate) != 0) {
      // Add measurements to particles
      particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
      particles[i].theta += yaw_rate * delta_t;
      
    } else {
      // Add measurements to particles
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
      // Theta will stay the same due to no yaw_rate
      
    }

    // Add noise to the particles
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
    
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, 
		std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to implement this method and use it as a helper during the updateWeights phase.
	// it's straitforward to understand association between senses observed landmarks and the real landmarks, but here the pridicted, what's this predicted landmark? 

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// Updates the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // First, when iterating through each particle, need to transform observation points to map coordinates.
  // Next, associate each observation to its nearest landmark. The distribution can then be calculated.
  
  // First term of multi-variate normal Gaussian distribution calculated below
  // It stays the same so can be outside the loop
  const double a = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
  
  // The denominators of the mvGd also stay the same
  const double x_denom = 2 * std_landmark[0] * std_landmark[0];
  const double y_denom = 2 * std_landmark[1] * std_landmark[1];

  // Iterate through each particle
  for (int i = 0; i < num_particles; ++i) {
    
    // For calculating multi-variate Gaussian distribution of each observation, for each particle
    double mvGd = 1.0;
    
    // For each observation
    for (int j = 0; j < observations.size(); ++j) {
      
      // Transform the observation point (from vehicle coordinates to map coordinates)
      double trans_obs_x, trans_obs_y;
      trans_obs_x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
      trans_obs_y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;
      
      // Find nearest landmark
      vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
      vector<double> landmark_obs_dist (landmarks.size());
      for (int k = 0; k < landmarks.size(); ++k) {
        
        // Down-size possible amount of landmarks to look at by only looking at those in sensor range of the particle
        // If in range, put in the distance vector for calculating nearest neighbor
        double landmark_part_dist = sqrt(pow(particles[i].x - landmarks[k].x_f, 2) + pow(particles[i].y - landmarks[k].y_f, 2));
        if (landmark_part_dist <= sensor_range) {
          landmark_obs_dist[k] = sqrt(pow(trans_obs_x - landmarks[k].x_f, 2) + pow(trans_obs_y - landmarks[k].y_f, 2));

        } else {
          // Need to fill those outside of distance with huge number, or they'll be a zero (and think they are closest)
          landmark_obs_dist[k] = 999999.0;
          
        }
        
      }
      
      // Associate the observation point with its nearest landmark neighbor
      int min_pos = distance(landmark_obs_dist.begin(),min_element(landmark_obs_dist.begin(),landmark_obs_dist.end()));
      float nn_x = landmarks[min_pos].x_f;
      float nn_y = landmarks[min_pos].y_f;
      
      // Calculate multi-variate Gaussian distribution
      double x_diff = trans_obs_x - nn_x;
      double y_diff = trans_obs_y - nn_y;
      double b = ((x_diff * x_diff) / x_denom) + ((y_diff * y_diff) / y_denom);
      mvGd *= a * exp(-b);
      
    }
    
    // Update particle weights with combined multi-variate Gaussian distribution
    particles[i].weight = mvGd;
    weights[i] = particles[i].weight;

  }
}

void ParticleFilter::resample() {
	// Resamples particles with replacement with probability proportional to their weight.
  
  // Vector for new particles
  vector<Particle> new_particles (num_particles);
  
  // Use discrete distribution to return particles by weight
  random_device rd;
  default_random_engine gen(rd());
  for (int i = 0; i < num_particles; ++i) {
    discrete_distribution<int> index(weights.begin(), weights.end());
    new_particles[i] = particles[index(gen)];
    
  }
  
  // Replace old particles with the resampled particles
  particles = new_particles;

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
	cout << "get associations start" << endl;
	vector<int> v = best.associations;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
	cout << "get associations end" << endl;
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	cout << "get getSenseX start" << endl;
	vector<double> v = best.sense_x;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  cout << "get getSenseX end" << endl;
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	cout << "get getSenseY start" << endl;
	vector<double> v = best.sense_y;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  cout << "get getSenseY end" << endl;
  return s;
}
