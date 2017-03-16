
#define MAXN WORDSIZE
#define ONE_WORD_SETS 1
//the above is especially used since our graphs will have small order, see nauty documentation

#include <iostream>
#include <fstream>
#include <queue>
#include <unistd.h>
#include <thread>

#include "Configuration.h"
#include "Bank.h"
#include "animation.h"

Configuration initialCluster();
void breakContactsAndAdd(Configuration& current, std::queue<Configuration>& Queue);
void enumerateClusters(Configuration* initial, Bank* bank);

/*	TODO NOW:
 *
 *	Clean up code, compartmentalize
 *	Implement lighting to make animation nicer
 *	Include drawing of rods in framework in animation
 *	Comment code, including animation code
 *	update makefile
 *
 */

//LATER Consider fixed size vectorization (16 byte alignment) via Eigen
//NOTE: nauty "graph" is just unsigned long (bitwise adj matrix)


int main(int argc, char** argv){
	
	Animation animation;
	Configuration::counter = 0;
	
	//Start with one cluster, built by iteratively adding one vertex with three edges to a triangle
	Configuration c = initialCluster();
	Bank* bank = new Bank();
	

	if(argc>1){
		animation.setup();
	
				//Set the initial position data for the animation
		ConfigVector p = c.getP();
		animation.setP(p);
	
		//Set the initial graph data for the animation
		graph* g = c.getG();
		animation.setG(g);
	
		// By giving the configuration class a pointer to the animation object, methods from within
		// that class may pass data to the animation (this is necessary in the walk method)
		Configuration::animation = &animation;
	
		
		/*
		 We take a multithreaded approach - the main thread runs the animation, and the second thread
		runs the program which enumerates the clusters.
		 */
		std::thread enumerate(enumerateClusters, &c, bank);
		animation.draw();
		enumerate.join();
	}else{
		enumerateClusters(&c, bank);
	}
	return 0;
	
	
}



void enumerateClusters(Configuration* initial, Bank* bank){
	std::queue<Configuration> Queue;
	Queue.push(*initial);
	Configuration current;
	while(Queue.size() > 0){
		current = Queue.front();
		std::cout<<"POP"<<std::endl;
		Queue.pop();
		
		if(!bank->add(current)){ //returns 0 if already in bank, otherwise adds and returns 1
			continue;
		}
		breakContactsAndAdd(current, Queue);
		
	}
	
	
	
}







Configuration initialCluster(){
	
	double points[NUM_OF_SPHERES*3];
	
	
	std::ifstream clusterFile;
	clusterFile.open ("first_cluster8.txt");
	if (clusterFile.is_open()){
		for(int i=0; i<NUM_OF_SPHERES*3; i++){
			clusterFile>>points[i];
		}
		
		clusterFile.close();
	}else{
		std::cout<<"Failed to open initial cluster file!"<<std::endl;
		return Configuration();
	}

	
	
	graph g[NUM_OF_SPHERES];// = (graph*) malloc(NUM_OF_SPHERES*sizeof(graph));
	memset(&g, 0, NUM_OF_SPHERES*sizeof(graph));
	
	Configuration* c = new Configuration(points, g);
	
	
	for(int i=0; i<5; i++){
		for(int j=1;j<4;j++){
			c->addEdge(i, i+j);
		}
	}
	c->addEdge(5, 6);
	c->addEdge(5, 7);
	c->addEdge(6, 7);
	return *c;
}

void breakContactsAndAdd(Configuration& current, std::queue<Configuration>& Queue){
	
	//TODO include lookup table to reduce redundancy?
	
	int dim;
	Configuration copy;
	std::vector<Configuration> walkedTo;
	
	//This double for-loop iterates through all edges in the graph of current
	for(int i=0; i<NUM_OF_SPHERES; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if( !current.hasEdge(i,j) ){
				continue;
			}
			
			// We make a copy, delete an edge of that copy, then either enter that copy into
			// the queue, or delete it, depending on whether it is rigid.
			copy = current.makeCopy();
			copy.deleteEdge(i,j);
			
			dim = copy.dimensionOfTangentSpace(false);
			if(dim == 0){
				Queue.push(copy);
				breakContactsAndAdd(copy, Queue);
			}
			else if(dim == 1){
				
				graph* g = copy.getG();
				if(Configuration::animation) Configuration::animation->setG(g);
				
				walkedTo = copy.walk();
				for(int k=0; k<walkedTo.size(); k++){ //walking can fail!
					walkedTo[k].canonize(); //walking added an edge!
					Queue.push(walkedTo[k]);
				}
			}
		}
	}
	
}






