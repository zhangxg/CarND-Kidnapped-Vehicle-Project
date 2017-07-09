//============================================================================
// Name        : bayesianFilter.cpp
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//constructor:
bayesianFilter::bayesianFilter() {

    //TODO add is_initialized to header file
    //set initialization to false:
    // NOTE: helps us set up the initial believe state
	is_initialized_ = false;

    //TODO add control_std to header file
	//set standard deviation of control to 1.0f:
	control_std = 1.0f;

	//define size of believe, same size as map
	bel_x.resize(100,0);
	
	//TODO add bel_x_init to header file
	// NOTE: helps us not overwrite believe during
	// the motion calculation
	// bel_x_init = false;
	bel_x_init.resize(100,0);

	observation_std = 1.0f;

}

//de-constructor:
bayesianFilter::~bayesianFilter() {

}

void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
        						             const map &map_1d,
                                         help_functions &helpers){

	/******************************************************************************
	 *  Set init belief of state vector:
	 ******************************************************************************/
	if(!is_initialized_){

		// TODO: run over map, all map_1d.lanmark_list values:
		for (int l=0; l< map_1d.landmark_list.size(); ++l){
			// TODO: get landmark l from map 
			map::single_landmark_s lm = map_1d.landmark_list[l];

			// check, if landmark position x fits in map [0,100]:
			// if(lm.id_i >= 0 && lm.id_i <= 100){
			if(lm.x_f > 0 && lm.x_f < bel_x_init.size()){
				// TODO: get landmark x position * use help_function.h for reference
				// ______/---\_____ << initial believe state at a landmark
				// TODO: set belief to 1 at position and +/- from position:
				//cast float to int:
				int position_x = int(lm.x_f);
				//set belief to 1:
				bel_x_init[position_x]   = 1.0f;
				bel_x_init[position_x-1] = 1.0f;
				bel_x_init[position_x+1] = 1.0f;
			} //end if

			// why do it this way, how about below? why check if ? why +/- position?
			// bel_x_init[l] = 1.0f;

		}//end for

		// TODO: normalize initial believe * use help_function.h for reference
		bel_x_init = helpers.normalize_vector(bel_x_init);

		// set initial flag to true:
		is_initialized_ = true;
	
	}//end if
	
	/******************************************************************************
	 *  motion model and observation update
	******************************************************************************/
	std::cout <<"-->motion model for state x ! \n" << std::endl;

	//get current observations and control information:
	MeasurementPackage::control_s     controls = measurements.control_s_;
	MeasurementPackage::observation_s observations = measurements.observation_s_;

	// run over all bel_x values (index represents the pose in x!):
	for (int i=0; i<bel_x.size(); ++i){

		float pose_i = float(i) ;
		/**************************************************************************
		 *  posterior for motion model
		**************************************************************************/
        // motion posterior:
        // used to set up the convolution
		float posterior_motion = 0.0f;

		//loop over state space x_t-1 * same size as bel_x (Perform Convolution):
		for (int j=0; j<bel_x.size(); ++j){
			
			//TODO: Calculate transition probabilites using helpers.normpdf()
			// x: difference between bel_x index and state space index
			// mu: the movement from controls defined above
			// std: defined eariler
			//TODO: Calculate motion model
			// ADD the transition prob multiplied by the intial believe 
			// at state space index
			//posterior_motion +=
			float pose_j = float(j) ;
			float distance_ij = pose_i-pose_j;
			//transition probabilities:
			float transition_prob = helpers.normpdf(distance_ij,
                    								controls.delta_x_f,
                    								control_std) ;
			//motion model:
			posterior_motion += transition_prob*bel_x_init[j];
		}


		/**************************************************************************
		 *  observation update:
		**************************************************************************/
		//define pseudo observation vector:
		std::vector<float> pseudo_ranges ;

		//define maximum distance:
		float distance_max = 100;
			
		//loop over number of landmarks and estimate pseudo ranges:
		for (unsigned int l=0; l<map_1d.landmark_list.size(); ++l){

			//calculate difference between landmark position
			// and current believe state index
			float range_l = map_1d.landmark_list[i].x_f - pose_i;
			
			//check, if distances are positive, and store positive range: 
			if(range_l > 0) {
				pseudo_ranges.push_back(range_l) ;	
			}
		}

		//sort pseudo range vector:
		sort(pseudo_ranges.begin(), pseudo_ranges.end());

		//define observation posterior:
		float posterior_obs = 1.0f ;
		
		//run over current observations vector defined above:
		for (unsigned int z=0; z<observations.distance_f.size(); ++z){

			//define min distance:
			// TODO: set min distance either to the closet landmark
			// or if no landmarks exist to the maximum set distance
			//estimate the posterior for observation model: 
			// MULTIPLY by normpdf of obseravations distance, 
			// min distance, and obseravtion_std
			//posterior_obs*= 

			//define min distance:
			float pseudo_range_min;

			//check, if distance vector exists:
			if(pseudo_ranges.size() > 0){

				//set min distance:
				pseudo_range_min = pseudo_ranges[0];
				//remove this entry from pseudo_ranges-vector:
				pseudo_ranges.erase(pseudo_ranges.begin());

			}
			//no or negative distances: set min distance to maximum distance:
			else{
				pseudo_range_min = distance_max ;
			}

			//estimate the posterior for observation model: 
			posterior_obs*= helpers.normpdf(observations.distance_f[z], 
											pseudo_range_min,
											observation_std); 
		}

		//TODO: update = motion_model 
		// set as the posterior_motion
		// bel_x[i] = posterior_motion;

		/**************************************************************************
		 *  finalize bayesian localization filter:
		 *************************************************************************/
		
		//update = observation_update* motion_model
		bel_x[i] = posterior_obs*posterior_motion ;
	};
	//TODO: normalize bel_x:
	bel_x = helpers.normalize_vector(bel_x);

	//TODO: set initial believe to bel_x for next time
	bel_x_init = bel_x;
};