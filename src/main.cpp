
#define MAXN WORDSIZE
#define ONE_WORD_SETS 1
//the above is specially used since our graphs will have small order, see nauty documentation

#include <iostream>
#include <queue>
#include "Configuration.h"
#include "Bank.h"


//TODO Consider fixed size vectorization (16 byte alignment) via Eigen
Configuration* firstCluster(){
	
	float points[NUM_OF_SPHERES*3];
	for(int i=0; i<NUM_OF_SPHERES*3; i++){
		points[i] = 0;
	}
	
	Configuration* c = new Configuration(points);
	
	graph* g = (graph*) malloc(NUM_OF_SPHERES*sizeof(graph));
	c->addGraph(g);
	
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

int main(){
	
	//BEGIN TESTING
	
	return 0;
	//END TESTING
	
	//INITIALIZATION
	std::queue<Configuration*> Queue;
	Bank* bank = new Bank();
	
	Queue.push(firstCluster());

	Configuration* current;
	while(Queue.size() > 0){
		current = Queue.front();
		Queue.pop();
		
		current->canonize();
		if(add(current)){ //returns 1 if already in bank, otherwise adds and returns 0
			continue;
		}
		
	}
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


//FIXME
