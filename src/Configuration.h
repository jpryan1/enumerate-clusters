#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 3

#include <iostream>
#include "Eigen/Dense"
#include "nauty.h"

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
	void addGraph(graph* add){
		g = add;
	}
	void printDetails();
	
	
	int compareGraph(Configuration* other); //comparator for tree

	void canonizeGraph();
	
	void deleteEdge(int i, int j);
	void addEdge(int i, int j);
	
private:
	ConfigVector* p; // contains 3*n floats for points in space
	graph* g;
	

	
};




#endif




