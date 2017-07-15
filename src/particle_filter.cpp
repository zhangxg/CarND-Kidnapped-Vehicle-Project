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

std::vector<int> associate(std::vector<LandmarkObs> observations, Map map_landmarks) {
	std::vector<int> associated;
	for (int j = 0; j < observations.size(); ++j) {
		LandmarkObs o = observations[j];
		std::vector<double> dists;
		for (int i = 0; i < map_landmarks.landmark_list.size(); ++i) {
			Map::single_landmark_s lm = map_landmarks.landmark_list[i];
			dists.push_back(sqrt((o.x-lm.x_f)*(o.x-lm.x_f) + (o.y-lm.y_f)*(o.y-lm.y_f)));
		}
		std::vector<double>::iterator min_result;
		min_result = min_element(dists.begin(), dists.end());
		associated.push_back(distance(dists.begin(), min_result));
	}
	return associated;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 100;
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]); //in radian
	for (int i = 0; i < num_particles; ++i)
	{
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;
		particles.push_back(particle);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	double x, y, theta;
	std::vector<double> s;
	for (int i = 0; i < num_particles; ++i)
	{
		s[0] = particles[i].x;
		s[1] = particles[i].y;
		s[2] = particles[i].theta;

		x = s[0] + velocity / yaw_rate * (sin(s[2] + yaw_rate * delta_t) - sin(s[2]));
		y = s[1] + velocity / yaw_rate * (cos(s[2]) - cos(s[2] + yaw_rate * delta_t));
		theta = s[2] + yaw_rate * delta_t;

		normal_distribution<double> dist_x(x, std_pos[0]);
		normal_distribution<double> dist_y(y, std_pos[1]);
		normal_distribution<double> dist_theta(theta, std_pos[2]);
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
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
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. 
	// You can read more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for (int i = 0; i < num_particles; ++i)
	{
		// do transformation: convert the car coordinates to the map coordinates;
		std::vector<LandmarkObs> transed;
		for (int j = 0; j < observations.size(); ++j)
		{
			transed.push_back(transformCoordinate(observations[j], particles[i]));
		}

		// do association: associate the transformed observation with the real landmarks, using the nearest neighbour
		particles[i].associations = associate(transed, map_landmarks);

		//calculate weights
		
			
	}

	


	// weight


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
