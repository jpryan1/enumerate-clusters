#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 11

#include <iostream>
#include "Eigen/Dense"
#include "nausparse.h"

using namespace Eigen;

class Configuration{
	
public:
	Configuration(int num_of_spheres, float* points){
		//obviously graph should be initialized here too, but we'll worry about that later
		p = (VectorXf*) malloc(num_of_spheres*sizeof(float)*3);
		memcpy(p, points, num_of_spheres*sizeof(float)*3);
		
	}
	~Configuration(){
		free(p);
	}
//	void addGraph(sparsegraph* add){
//		graph = add;
//	}
//	void printDetails(){
//		std::cout<<"PRINTING"<<std::endl;
//		for(int i=0; i<graph->nv; i++){
//			std::cout<<graph->d[i]<<std::endl;
//		}std::cout<<"DONE PRINTING"<<std::endl;
//	}

	
	
	
	int compareTo(Configuration* other); //comparator for tree

	void canonizeGraph();
	
	
private:
	VectorXf* p; // contains 3*n floats for points in space
	sparsegraph* graph;
	

	
};




#endif




