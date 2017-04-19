
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
#include "Timer.h"
Configuration initialCluster();
void breakContactsAndAdd(Configuration& current, std::queue<Configuration>& Queue, Bank& predim);
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
Timer timer;

int main(int argc, char** argv){
	
	Animation animation;
	
	//Start with one cluster, built by iteratively adding one vertex with three edges to a triangle
	Configuration c = initialCluster();
	Bank* bank = new Bank();
//	Configuration* d = (Configuration*) malloc(sizeof(c));;
//	memcpy(d, &c, sizeof(c));
//	std::cout<<d->matches(c)<<std::endl;
//	d->fixTriangle();
//	std::cout<<d->matches(c)<<std::endl;
////	std::cout<<c.matches(*d)<<std::endl;
////	std::cout<<c.matches(*d)<<std::endl;
//	free(d);
//	return 0;

	
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
	//bank->printDetails();
	return 0;
	
	
}



void enumerateClusters(Configuration* initial, Bank* bank){
	double t = time(0);
	std::queue<Configuration> Queue;
	Queue.push(*initial);
	Configuration current;
	int hyper = 1;
	int hypo = 1;
	int total = 0;
	Bank predim;
	while(Queue.size() > 0){
		current = Queue.front();
		Queue.pop();
		current.canonize();
		if(!bank->add(current)){ //returns 0 if already in bank, otherwise adds and returns 1
			continue;
		}total++;
			if(current.num_of_contacts<3*NUM_OF_SPHERES-6){
			std::cout<<"Just added hypostatic to bank "<<hypo++<<" "<<current.num_of_contacts<<std::endl;
			
//			if(Configuration::animation){
//				current.show(9);
//			}

		}
		if(current.num_of_contacts>3*NUM_OF_SPHERES-6){
			//current.show(4);
			std::cout<<"Just added hyperstatic to bank "<<hyper++<<" "<<current.num_of_contacts<<std::endl;
		}
		breakContactsAndAdd(current, Queue, predim);
		
	}
	
	
	
	std::cout<<"Bank is size "<<bank->size()<<std::endl;
	std::cout<<"Aux banks are "<<predim.size()<<std::endl;
	t = time(0) - t;
	std::cout<<"Elapsed time: "<<t<<" seconds."<<std::endl;
	timer.display();
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

	
	
	graph g[NUM_OF_SPHERES];
	memset(g, 0, NUM_OF_SPHERES*sizeof(graph));
	
	Configuration* c = new Configuration(points, g, false);
	
	
	for(int i=0; i<NUM_OF_SPHERES-3; i++){
		for(int j=1;j<4;j++){
			c->addEdge(i, i+j);
		}
	}
	c->addEdge(NUM_OF_SPHERES-3, NUM_OF_SPHERES-2);
	c->addEdge(NUM_OF_SPHERES-3, NUM_OF_SPHERES-1);
	c->addEdge(NUM_OF_SPHERES-2, NUM_OF_SPHERES-1);
	
	c->canonize();
	return *c;
}

void breakContactsAndAdd(Configuration& current, std::queue<Configuration>& Queue, Bank& predim ){
	
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
			copy = current.makeCopy(false);
			copy.deleteEdge(i,j);
			copy.canonize();
			if(!predim.add(copy)){
				continue;
			}
			dim = copy.dimensionOfTangentSpace(true);
			
			
			if(dim == 0){
				breakContactsAndAdd(copy, Queue, predim);
			}
			
			else if(dim == 1){
				walkedTo = copy.walk();
				for(int k=0; k<walkedTo.size(); k++){ //walking can fail!
					int temp =walkedTo[k].dimensionOfTangentSpace(false);
					if(temp>0) continue;
					
					Queue.push(walkedTo[k]);
				}
			}
		}
	}
	
}






