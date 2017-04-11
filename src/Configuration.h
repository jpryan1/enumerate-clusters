#ifndef  _CONFIGURATION_H_    /* only process this file once */
#define  _CONFIGURATION_H_

#define NUM_OF_SPHERES 10

#include <iostream>
#include "Eigen/Dense"
#include "nauty.h"
#include "animation.h"
#include <vector>
#include <bitset>
#include <ctime>

using namespace Eigen;
typedef Matrix<double, 3*NUM_OF_SPHERES, 1> ConfigVector;

typedef std::pair <int, int> Contact;



class Configuration{
public:
	
	static Animation* animation;
	
	
	Configuration(){}
	Configuration(double* points, graph* adj,  bool setTriangle = true);
	~Configuration(){}
	
	Configuration(const Configuration& other){
		num_of_contacts = other.num_of_contacts;
		memcpy(&p, &other.p, 3*NUM_OF_SPHERES*sizeof(double));
		memcpy(g, other.g, NUM_OF_SPHERES*sizeof(graph));
		memcpy(triangle, other.triangle, 3*sizeof(int));
	}
	
	
	void chooseTriangle();
	void fixTriangle();
	
	Configuration makeCopy(bool setTriangle = true);
	void printDetails();
	
	
	int compareGraph(Configuration& other); //returns true if graphs match
	
	int matches(Configuration& other); //returns true if configurations are the same
	int matchesHelper(Configuration& other);

	void canonize();
	
	void deleteEdge(int i, int j);
	void addEdge(int i, int j);
	
	int hasEdge(int i, int j);
	
	int dimensionOfTangentSpace(bool useNumericalMethod);
	int numerical_findDimension(MatrixXd& right_null_space);
	
	void project(ConfigVector& old, ConfigVector& proj);
	void project();
	void populate_F_vec(ConfigVector& initial, MatrixXd& F_vec);
	
	std::vector<Contact> checkForNewContacts(ConfigVector proj, bool smallTol = false);
	void populateRigidityMatrix(MatrixXd& rigid, ConfigVector& points);
	std::vector<Configuration> walk();
	
	ConfigVector getP();
	graph* getG();
	void setTriangle(int* t){
		memcpy(triangle, t, 3*sizeof(int));
	}
	int num_of_contacts;
	void checkTriangle(int a);
	void show(int a);
	
private:
	int triangle[3];
	bool isRegular;
	ConfigVector p;
	// contains 3*n doubles for points in space
	ConfigVector v;
	graph g[NUM_OF_SPHERES];
	
	
};




#endif




