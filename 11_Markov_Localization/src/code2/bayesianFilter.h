//============================================================================
// Name        : bayesianFilter.h
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#ifndef BAYESIANFILTER_H_
#define BAYESIANFILTER_H_

#include <vector>
#include <string>
#include <fstream>

#include "measurement_package.h"
#include "map.h"
#include "help_functions.h"

class bayesianFilter {
public:

	// bool is_initialized_;

	// float control_std;

	// bool bel_x_init;

	//constructor:
	bayesianFilter();
	//deconstructor:
	virtual ~bayesianFilter();


	//main public member function, which estimate the beliefs:
	void process_measurement(const MeasurementPackage &measurements,
			                 const map &map_1d,
			                 help_functions &helpers);

	//member public: belief of state x:
	std::vector<float> bel_x ;

private:

/////private members:

	////////////////
    //Add members!//
    ////////////////

	// the lecture defines these as private, while i in public;
	bool is_initialized_;

	float control_std;

	// bool bel_x_init;
	std::vector<float> bel_x_init;

};

#endif /* BAYESIANFILTER_H_ */
