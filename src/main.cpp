


#include <iostream>
#include <queue>
#include "Configuration.h"
#include "Bank.h"


//TODO Consider fixed size vectorization (16 byte alignment) via Eigen

sparsegraph* genGraph(){
	sparsegraph* gg = new sparsegraph();
	gg->nv = 3;
	gg->nde = 4;
	SG_ALLOC(*gg, gg->nv, gg->nde, "ALLOC FAILED");
	
	gg->v[0] = 0;
	gg->v[1] = 2;
	gg->v[2] = 3;
	
	gg->e[0] = 1;
	gg->e[1] = 2;
	gg->e[2] = 0;
	gg->e[3] = 0;
	
	gg->d[0] = 2;
	gg->d[1] = 1;
	gg->d[2] = 1;
	return gg;
}

sparsegraph* genGraph1(){
	sparsegraph* gg = new sparsegraph();
	gg->nv = 3;
	gg->nde = 2;//4;
	SG_ALLOC(*gg, gg->nv, gg->nde, "ALLOC FAILED");
	
	gg->v[0] = 0;
	gg->v[1] = 1;
	gg->v[2] = 2;//3;
	
	gg->e[0] = 1;
	gg->e[1] = 0;
	//gg->e[2] = 2;
	//gg->e[3] = 1;
	
	gg->d[0] = 1;
	gg->d[1] = 1;//2;
	gg->d[2] = 0;//1;
	return gg;
}


//NOTE, when we begin a walk,
int main(){
	
	//BEGIN TESTING
	std::cout<<"\n\n\nTESTING:\n\n"<<std::endl;
	sparsegraph* g = genGraph();
	sparsegraph* g1 = genGraph1();
	float points[9];
	for(int i=0;i<9;i++) points[i] = 0;
	points[0] = 0; points[3] = 1; points[7] = 1;
	Configuration* gg = new Configuration(points);
	Configuration* gg1 = new Configuration(points);
	gg->addGraph(g);
	gg1->addGraph(g1);
	
	
	Bank* bank = new Bank();
	gg->canonizeGraph();
	gg1->canonizeGraph();
	bank->add(gg);
	bank->add(gg1);
	//bank->printDetails();
	
	
	std::queue<Configuration*> Queue;
	Queue.push(gg);
	Queue.push(gg1);
	Queue.front()->printDetails();
	Queue.pop();
	std::cout<<Queue.size();
	std::cout<<"\n\n\nALL GOOD!\n\n\n"<<std::endl;
	
	
	//END TESTING
	
	//INITIALIZATION
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
