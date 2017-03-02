
#define MAXN WORDSIZE
#define ONE_WORD_SETS 1
//the above is specially used since our graphs will have small order, see nauty documentation

#include <iostream>
#include <fstream>
#include <queue>
#include "Configuration.h"
#include "Bank.h"


Configuration* initialCluster();
void breakContactsAndAdd(Configuration* current, std::queue<Configuration*> Queue);


//TODO NOW: Implement dimensionOfTangentSpace() and walk()

//LATER Consider fixed size vectorization (16 byte alignment) via Eigen
//NOTE: nauty "graph" is just unsigned long (bitwise adj matrix)


int main(){
	
	
	//BEGIN TESTING
	
	//INITIALIZATION
	Configuration* c = initialCluster();
	
	if(!c){
		return 1;
	}
	c->deleteEdge(0,1);
	std::cout<<c->dimensionOfTangentSpace(true)<<std::endl;
	
//	delete c;
//	d->printDetails();
//	delete d;
	return 0;
	
//	c->canonize();
//	std::queue<Configuration*> Queue;
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


Configuration* initialCluster(){
	
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
		return NULL;
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
	return c;
}

void breakContactsAndAdd(Configuration* current, std::queue<Configuration*> Queue){
	
	//TODO include lookup table to reduce redundancy?
	
	int dim;
	Configuration* copy;
	for(int i=0; i<NUM_OF_SPHERES; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if( !current->hasEdge(i,j) ){
				continue;
			}
			copy = current->makeCopy();
			copy->deleteEdge(i,j);
			//copy->canonize(); Not necessary??
			
			dim = copy->dimensionOfTangentSpace(false);
			if(dim == 0){
				breakContactsAndAdd(copy, Queue);
				delete copy;
			}
			else if(dim == 1){
				if(copy->walk()){ //walking can fail!
					copy->canonize(); //walking added an edge!
					Queue.push(copy);
				}
			}
			else{
				delete copy;
			}
		}
	}
}






