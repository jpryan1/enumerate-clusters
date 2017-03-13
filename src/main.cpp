
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
void breakContactsAndAdd(Configuration current, std::queue<Configuration>& Queue);


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
	
	//if(argc>1){
		Animation animation;
		animation.setup();
	//}
	//BEGIN TESTING
	
	//INITIALIZATION
	Configuration c = initialCluster();
	
//	if(!c){
//		return 1;
//	}
	
	//c->deleteEdge(0,2);
	
	ConfigVector p = c.getP();
	animation.setP(p);
	
	graph* g = c.getG();
	animation.setG(g);
	
	
	std::queue<Configuration> Queue;
	
	//Commented out until bug fixed
	//std::thread walker(&breakContactsAndAdd, c, Queue);
	animation.draw();
//	walker.join();
	delete &c;
//	d->printDetails();
//	delete d;
	return 0;
	
//	c->canonize();
	
//	Bank* bank = new Bank();
//	
//	Queue.push(c);
//	
//	
//	
//
//	Configuration* current;
//	while(Queue.size() > 0){
//		current = Queue.front();
//		
//		Queue.pop();
//		
//		if(!bank->add(current)){ //returns 0 if already in bank, otherwise adds and returns 1
//			continue;
//		}
//		breakContactsAndAdd(current, Queue);
//	}
//	
	
	
	
	//START WITH ONE CLUSTER
	//CREATE EMPTY BANK
	//CREATE EMPTY QUEUE WITH FIRST CLUSTER
	
	//FOR EACH ITEM IN QUEUE
		//CONFIG = DEQUEUE()
		//SET TO CANONICAL FORM
		//IF ALREADY IN BANK
		//	CONTINUE
		//ELSE
		//	PUT IN BANK
		//	ITERATE THROUGH SUBSETS OF CONTACTS
		//		BREAK SUBSET
		//		CHECK DIMENSION OF TANGENT SPACE TO SOLUTION AT T=0
		//		IF DIM = 1,
		//			WALK ALONG, ARRIVE AT NEW CLUSTER //(check rigidity)
		//			ADD NEW CLUSTER TO QUEUE
		//		ENDIF
		//	ENDITERATE
		//ENDIF
		//
	//ENDFOR
	
	
	
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

void breakContactsAndAdd(Configuration current, std::queue<Configuration>& Queue){
	
	//TODO include lookup table to reduce redundancy?

	int dim;
	Configuration copy;
	std::vector<Configuration> walkedTo;
	for(int i=0; i<NUM_OF_SPHERES; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if( !current.hasEdge(i,j) ){
				continue;
			}
			copy = current.makeCopy();
			copy.deleteEdge(i,j);
			//copy->canonize(); Not necessary??
			graph* g = copy.getG();
			Configuration::animation.setG(g);
			dim = copy.dimensionOfTangentSpace(false);
			if(dim == 0){
				breakContactsAndAdd(copy, Queue);
				
			}
			else if(dim == 1){
				walkedTo = copy.walk();
				for(int k=0; k<walkedTo.size(); k++){ //walking can fail!
					walkedTo[k].canonize(); //walking added an edge!
					Queue.push(walkedTo[k]);
				}
			}
		}
	}
}






