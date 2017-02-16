#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 8

#include <iostream>
#include "Eigen/Dense"
#include "nauty.h"

using namespace Eigen;
typedef Matrix<double, 3*NUM_OF_SPHERES, 1> ConfigVector;

class Configuration{
	
public:
	Configuration(double* points, graph* adj){
		//obviously graph should be initialized here too, but we'll worry about that later
		
		memcpy(&p, points, 3*NUM_OF_SPHERES*sizeof(double));
	
		memcpy(g, adj, NUM_OF_SPHERES*sizeof(graph));
		
	}
	
	~Configuration(){}
	
	Configuration* makeCopy();
	void printDetails();
	
	
	int compareGraph(Configuration* other); //returns true if graphs match
	
	int matches(Configuration* other); //returns true if configurations are the same
	
	void canonize();
	
	void deleteEdge(int i, int j);
	void addEdge(int i, int j);
	
	int hasEdge(int i, int j);
	
	int dimensionOfTangentSpace(bool useNumericalMethod);
	int numerical_findDimension();
	void populateRigidityMatrix(MatrixXd& rigid, ConfigVector& points);
	int walk();
	
	
private:
	
	ConfigVector p; // contains 3*n doubles for points in space
	ConfigVector v;
	graph g[NUM_OF_SPHERES];
	
	
};




#endif




