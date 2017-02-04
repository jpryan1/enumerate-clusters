#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 3

#include <iostream>
#include "Eigen/Dense"
#include "nausparse.h"

using namespace Eigen;

typedef Matrix<float, 3*NUM_OF_SPHERES, 1> ConfigVector;

class Configuration{
	
public:
	Configuration(float* points){
		//obviously graph should be initialized here too, but we'll worry about that later
		
		p = new ConfigVector();
		memcpy(p, points, 3*NUM_OF_SPHERES*sizeof(float));
		
	}
	
	~Configuration(){
		delete p;
	}
	void addGraph(sparsegraph* add){
		graph = add;
	}
	void printDetails();
	
	
	int compareGraph(Configuration* other); //comparator for tree

	void canonizeGraph();
	
	
private:
	ConfigVector* p; // contains 3*n floats for points in space
	sparsegraph* graph;
	

	
};




#endif




