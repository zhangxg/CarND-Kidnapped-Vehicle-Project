// #include "particle_filter.h"
// #include "particle_filter.cpp"
#include <iostream>
#include <vector>
#include <math.h>
#include "map.h"
#include <iterator>

using namespace std;

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
	
	int id;				// Id of matching landmark in the map.
	double x;			// Local (vehicle coordinates) x position of landmark observation [m]
	double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

struct Particle {
	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};

void printIntVector(std::vector<int> v) {
	for (int i = 0; i < v.size(); ++i)
	{
		cout << v[i] << ",  ";
	}
	cout << endl;
}

void printDoubleVector(std::vector<double> v, bool isScientific) {
	for (int i = 0; i < v.size(); ++i)
	{
		if (isScientific) {
			cout << scientific;
			cout << v[i] << ",  ";	
		} else {
			cout << v[i] << ",  ";	
		}
	}
	cout << endl;
}

// std::vector<double> transformCoordinate(std::vector<double> obs, std::vector<double> particle) {

// 	cout << "call the function" << endl;
// 	// obs: the observation car coordinates, [x, y, heading]
// 	// particle: the particle map coordinates
// 	double radian_obs_in_car = atan2(obs[1], obs[0]);
// 	cout << radian_obs_in_car << endl;
// 	double radian_obs_in_map = radian_obs_in_car + obs[2];
// 	cout << radian_obs_in_map << endl;

// 	std::vector<double> transed;
// 	double distance = sqrt(obs[0]*obs[0] + obs[1]*obs[1]);
// 	cout << distance << endl;
// 	// transed[0] = particle[0] + distance * cos(radian_obs_in_map);
// 	// transed[1] = particle[1] + distance * sin(radian_obs_in_map);
// 	transed.push_back(particle[0] + distance * cos(radian_obs_in_map));
// 	transed.push_back(particle[1] + distance * sin(radian_obs_in_map));
// 	return transed;
// }

// std::vector<double> transformCoordinate(LandmarkObs& obs, Particle& particle) {

// 	double radian_obs_in_car = atan2(obs.y, obs.x);
// 	cout << radian_obs_in_car << endl;
// 	double radian_obs_in_map = radian_obs_in_car + particle.theta;
// 	cout << radian_obs_in_map << endl;

// 	std::vector<double> transed;
// 	double distance = sqrt(obs.x * obs.x + obs.y * obs.y);
// 	cout << distance << endl;
// 	// transed[0] = particle[0] + distance * cos(radian_obs_in_map);
// 	// transed[1] = particle[1] + distance * sin(radian_obs_in_map);
// 	transed.push_back(particle.x + distance * cos(radian_obs_in_map));
// 	transed.push_back(particle.y + distance * sin(radian_obs_in_map));

// 	obs.x = particle.x + distance * cos(radian_obs_in_map);
// 	obs.y = particle.y + distance * sin(radian_obs_in_map);

// 	return transed;
// }

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
	// it's not correct, it's associate landmark to observation, not vise vurs.
	// for (int i = 0; i < map_landmarks.landmark_list.size(); ++i)
	// {
	// 	Map::single_landmark_s lm = map_landmarks.landmark_list[i];
	// 	std::vector<double> distances;
	// 	for (int j = 0; j < observations.size(); ++j)
	// 	{
	// 		LandmarkObs o = observations[j];
	// 		distances.push_back(sqrt((o.x-lm.x_f)*(o.x-lm.x_f) + (o.y-lm.y_f)*(o.y-lm.y_f)));
	// 	}
	// 	for (int j = 0; j < observations.size(); ++j)
	// 	{
	// 		cout << distances[j] << ",";
	// 	}
	// 	cout << endl;
	// }
	std::vector<int> associated;
	for (int j = 0; j < observations.size(); ++j)
	{
		LandmarkObs o = observations[j];
		std::vector<double> dists;
		for (int i = 0; i < map_landmarks.landmark_list.size(); ++i)
		{
			Map::single_landmark_s lm = map_landmarks.landmark_list[i];
			dists.push_back(sqrt((o.x-lm.x_f)*(o.x-lm.x_f) + (o.y-lm.y_f)*(o.y-lm.y_f)));
		}
		// for (int j = 0; j < map_landmarks.landmark_list.size(); ++j)
		// {
		// 	cout << dists[j] << ",";
		// }
		// cout << endl;
		// double* v2a = &dists[0];
		// double v2a[dists.size()];
		// copy(dists.begin(), dists.end(), v2a);
		std::vector<double>::iterator min_result;
		min_result = min_element(dists.begin(), dists.end());
		associated.push_back(distance(dists.begin(), min_result));
	}
	return associated;
}

void calculateWeight(std::vector<LandmarkObs> observations, Map map_landmarks, Particle& particle, double std_landmark[]) {
	double ox, oy, lx, ly, stdx, stdy;
	std::vector<double> weights;
	for (int i = 0; i < particle.associations.size(); ++i)
	{
		// std::vector<int> associated = particle.associations;
		// std::vector<double> weights;
		int landmark_index = particle.associations[i];
		ox = observations[i].x;
		oy = observations[i].y;
		stdx = std_landmark[0];
		stdy = std_landmark[1];
		// find the associated landmark coordinates
		for (int k = 0; k < map_landmarks.landmark_list.size(); ++k)
		{
			if (landmark_index == map_landmarks.landmark_list[k].id_i) {
				lx = map_landmarks.landmark_list[k].x_f;
				ly = map_landmarks.landmark_list[k].y_f;
			}
			// double weight;
			// weight = exp(-0.5*((ox-lx)*(ox-lx)/(stdx*stdx) + (oy-ly)*(oy-ly)/(stdy*stdy))) / (2*M_PI*stdx*stdy);
			// weights.push_back(weight);
		}
		double weight;
		weight = exp(-0.5*((ox-lx)*(ox-lx)/(stdx*stdx) + (oy-ly)*(oy-ly)/(stdy*stdy))) / (2*M_PI*stdx*stdy);
		weights.push_back(weight);
		// printDoubleVector(weights, true);
		
		// for (int j = 0; j < observations.size(); ++j) {
		// 	ox = observations[j].x;
		// 	oy = observations[j].y;
		// 	// find the associated landmark coordinates
		// 	for (int k = 0; k < map_landmarks.landmark_list.size(); ++k)
		// 	{
		// 		if (landmark_index == map_landmarks.landmark_list[k].id_i) {
		// 			lx = map_landmarks.landmark_list[k].x_f;
		// 			ly = map_landmarks.landmark_list[k].y_f;
		// 		}
		// 	}

		// 	stdx = std_landmark[0];
		// 	stdy = std_landmark[1];
		// }
	}
	printDoubleVector(weights, true);
	double producted_wight = 1;
	for (int i = 0; i < weights.size(); ++i)
	{
		producted_wight *= weights[i];
	}
	particle.weight = producted_wight;
}

// TODO: make a map for easy access. 
Map initializeMap() {
	Map map;
  std::vector<Map::single_landmark_s> lm_list;
  Map::single_landmark_s lm1;
  lm1.id_i = 0;
  lm1.x_f = 5;
  lm1.y_f = 3;
  lm_list.push_back(lm1);
	Map::single_landmark_s lm2;
  lm2.id_i = 1;
  lm2.x_f = 2;
  lm2.y_f = 1;
  lm_list.push_back(lm2);
	Map::single_landmark_s lm3;
	lm3.id_i = 2;
  lm3.x_f = 6;
  lm3.y_f = 1;
	lm_list.push_back(lm3);
	Map::single_landmark_s lm4;
	lm4.id_i = 3;
  lm4.x_f = 7;
  lm4.y_f = 4;
  lm_list.push_back(lm4);
	Map::single_landmark_s lm5;
	lm5.id_i = 4;
  lm5.x_f = 4;
  lm5.y_f = 7;
  lm_list.push_back(lm5);
  map.landmark_list = lm_list;
  return map;
}

Particle initializeParticle() {
	Particle p;
	p.x = 4;
	p.y = 5;
	// p.theta = -90/180*M_PI;   // 90/180 = 0. this leads to calculation error;
	p.theta = -90.0/180*M_PI;
	return p;
}

std::vector<LandmarkObs> initializeObservations(Particle p) {
	LandmarkObs obs1, obs2, obs3;
  obs1.x = 2;
  obs1.y = 2;
  
  obs2.x = 3;
  obs2.y = -2;
  
  obs3.x = 0;
  obs3.y = -4;

  std::vector<LandmarkObs> observations;
  observations.push_back(transformCoordinate(obs1, p));
  observations.push_back(transformCoordinate(obs2, p));
  observations.push_back(transformCoordinate(obs3, p));

  return observations;
};

int main() {
	// -------------- TEST THE WIGHTS CALCULATION --------------
	double sigma_landmark [2] = {0.3, 0.3};
	Map mp = initializeMap();
	Particle p = initializeParticle();
	std::vector<LandmarkObs> observations = initializeObservations(p);
	p.associations = associate(observations, mp);
	printIntVector(p.associations);
	calculateWeight(observations, mp, p, sigma_landmark);
	cout << "WIGHTS:" << p.weight << endl;

	///*
	// -------------- TEST THE ASSOCIATION --------------
  // int A[10] = {0, 2, 3, 1, 10, 34, 4};
  // const int N = sizeof(A) / sizeof(int);
  // std::vector<double> A;
  // A.push_back(0.88778);
  // A.push_back(4.778998);
  // A.push_back(3.543);
  // A.push_back(3.90);
  // A.push_back(300.90);

  // //this works
  // std::vector<double>::iterator result;
  // result = max_element(A.begin(), A.end());
  // std::cout << "max element at: " << std::distance(A.begin(), result) << '\n'; 

  // cout << "Index of max element: "
  //      // << std::distance(A, max_element(begin(A), end(A)))
  // 				// << std::distance(begin(A), end(A))
  //      << max_element(A.begin(), A.end())
  //      << endl;

  // Map map = initializeMap();
  // Particle p = initializeParticle();
  // std::vector<LandmarkObs> observations = initializeObservations(p);

  // std::vector<int> v;
  // v = associate(observations, map);
  // for (int i = 0; i < v.size(); ++i)
  // {
  // 	cout << v[i] << endl;
  // }
	// */
	/*	
	// -------------- TEST THE TRANSFORMATION --------------
	// cout << "hello, world" << endl;
	// std::vector<double> o;
	// o.push_back(2);
	// o.push_back(2);
	// o.push_back(-90.0/180*M_PI);
	// // o[0] = 2;
	// // o[1] = 2;
	// // o[2] = -90/180*M_PI;
	// std::vector<double> p;
	// p.push_back(4);
	// p.push_back(5);
	// // p[0] = 4;
	// // p[1] = 5;

	LandmarkObs o;
	// o.x = 0;
	// o.y = -4;
	o.x = 2;
	o.y = 2;
	o.id = 1;


	Particle p;
	p.x = 4;
	p.y = 5;
	// p.theta = -90/180*M_PI;   // 90/180 = 0. this leads to calculation error;
	p.theta = -90.0/180*M_PI;

	// std::vector<double> t = transformCoordinate(o, p);
	LandmarkObs t;
	t = transformCoordinate(o, p);
	cout << "result==" << endl;
	// cout << t[0] << ", " << t[1] << endl;
	cout << t.x << ", " << t.y << ", " << t.id << endl;
	cout << o.x << ", " << o.y << endl;
	*/
	return 0;
}


// int main() 
// {
//     std::vector<double> v; //{ 3, 1, 4 };
//     v.push_back(3);
//     v.push_back(1);
//     v.push_back(4);
//     std::cout << "distance(first, last) = "
//               << std::distance(v.begin(), v.end()) << '\n'
//               << "distance(last, first) = "
//               << std::distance(v.end(), v.begin()) << '\n';
// }



