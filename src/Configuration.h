#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 8

#include <iostream>
#include "Eigen/Dense"
#include "nauty.h"

using namespace Eigen;
typedef Matrix<float, 3*NUM_OF_SPHERES, 1> ConfigVector;

class Configuration{
	
public:
	Configuration(float* points, graph* adj){
		//obviously graph should be initialized here too, but we'll worry about that later
		
		p = new ConfigVector();
		memcpy(p, points, 3*NUM_OF_SPHERES*sizeof(float));
	
		g = (graph*) malloc(NUM_OF_SPHERES*sizeof(graph));
		memcpy(g, adj, NUM_OF_SPHERES*sizeof(graph));
		
	}
	
	~Configuration(){
		delete p;
		free(g);
		//g is malloced (FOR NOW), p is allocated via a constructer
	}
	
	Configuration* makeCopy();
	void printDetails();
	
	
	int compareGraph(Configuration* other); //comparator for tree

	void canonize();
	
	void deleteEdge(int i, int j);
	void addEdge(int i, int j);
	
	int hasEdge(int i, int j);
	
	
	void walk();
private:
	ConfigVector* p; // contains 3*n floats for points in space
	graph* g;
	
};




#endif




