#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 8

#include <iostream>
#include "Eigen/Dense"
#include "nauty.h"
#include "animation.h"
#include <vector>


using namespace Eigen;
typedef Matrix<double, 3*NUM_OF_SPHERES, 1> ConfigVector;

typedef std::pair <int, int> Contact;



class Configuration{
public:
	
	Configuration(){}
	
	Configuration(double* points, graph* adj){
		
		memcpy(&p, points, 3*NUM_OF_SPHERES*sizeof(double));
	
		memcpy(g, adj, NUM_OF_SPHERES*sizeof(graph));
		
	}
	
	~Configuration(){}
	
	Configuration makeCopy();
	void printDetails();
	
	
	int compareGraph(Configuration* other); //returns true if graphs match
	
	int matches(Configuration* other); //returns true if configurations are the same
	
	void canonize();
	
	void deleteEdge(int i, int j);
	void addEdge(int i, int j);
	
	int hasEdge(int i, int j);
	
	int dimensionOfTangentSpace(bool useNumericalMethod);
	int numerical_findDimension(MatrixXd& right_null_space);
	
	void project(ConfigVector& old, ConfigVector& proj);
	void populate_F_vec(ConfigVector& initial, MatrixXd& F_vec);
	
	std::vector<Contact> checkForNewContacts(ConfigVector proj);
	void populateRigidityMatrix(MatrixXd& rigid, ConfigVector& points);
	std::vector<Configuration> walk(Animation* animation);
	
	ConfigVector getP();
	
private:
	ConfigVector p;
	// contains 3*n doubles for points in space
	ConfigVector v;
	graph g[NUM_OF_SPHERES];
	
	
};




#endif




