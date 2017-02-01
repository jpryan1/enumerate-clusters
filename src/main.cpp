


#include <iostream>
#include "Configuration.h"
#include "Bank.h"


//TODO Consider fixed size vectorization (16 byte alignment) via Eigen

int main(){
	
	
	//BEGIN TESTING
//	sparsegraph* gg = new sparsegraph();
//	
//	gg->nv = 3;
//	gg->nde = 4;
//	SG_ALLOC(*gg, gg->nv, gg->nde, "ALLOC FAILED");
//	
//	gg->v[0] = 0;
//	gg->v[1] = 2;
//	gg->v[2] = 3;
//	
//	gg->e[0] = 1;
//	gg->e[1] = 2;
//	gg->e[2] = 0;
//	gg->e[3] = 0;
//	
//	gg->d[0] = 2;
//	gg->d[1] = 1;
//	gg->d[2] = 1;
//	
//	float points[3];
//	Configuration* config = new Configuration(1, points);
//	
//	config->addGraph(gg);
//	std::cout<<gg->nv<<std::endl;
//	config->printDetails();
//	config->canonizeGraph();
//	config->printDetails();
//	std::cout<<"ALL GOOD!"<<std::endl;
	
	//END TESTING
	
	//INITIALIZATION
	
	//TODO OBJECT TO CONTAIN FOUND CLUSTERS - CALL IT BANK
	//CREATE EMPTY QUEUE WITH FIRST ITEM
	
	//FOR EACH ITEM IN QUEUE
		//CONFIG = DEQUEUE()
		//SET TO CANONICAL FORM
		//IF ALREADY IN BANK
		//	CONTINUE
		//ELSE
		//	ITERATE THROUGH SUBSETS OF CONTACTS
		//		BREAK SUBSET
		//		CHECK DIMENSION OF TANGENT SPACE TO SOLUTION AT T=0
		//		IF DIM = 1,
		//			WALK ALONG, ARRIVE AT NEW CLUSTER //(check rigidity)
		//			ADD NEW CLUSTER TO QUEUE
		//		ENDIF
		//	ENDITERATE
		//ENDIF
		//PUT IN BANK
	//ENDFOR
	
	
	
}
