


#include <iostream>
#include "Configuration.h"
#include "Bank.h"


//TODO Consider fixed size vectorization (16 byte alignment) via Eigen
sparsegraph* genGraph(){
	sparsegraph* gg = new sparsegraph();
	gg->nv = 4;
	gg->nde = 6;
	SG_ALLOC(*gg, gg->nv, gg->nde, "ALLOC FAILED");

	gg->v[0] = 0;
	gg->v[1] = 1;
	gg->v[2] = 3;
	gg->v[3] = 5;

	gg->e[0] = 1;
	gg->e[1] = 0;
	gg->e[2] = 2;
	gg->e[3] = 1;
	gg->e[4] = 3;
	gg->e[5] = 2;

	gg->d[0] = 1;
	gg->d[1] = 2;
	gg->d[2] = 2;
	gg->d[3] = 1;
	return gg;
}

sparsegraph* genGraph1(){
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

sparsegraph* genGraph2(){
	sparsegraph* gg = new sparsegraph();
	gg->nv = 4;
	gg->nde = 6;
	SG_ALLOC(*gg, gg->nv, gg->nde, "ALLOC FAILED");
	
	gg->v[0] = 0;
	gg->v[1] = 3;
	gg->v[2] = 4;
	gg->v[3] = 5;
	
	gg->e[0] = 1;
	gg->e[1] = 2;
	gg->e[2] = 3;
	gg->e[3] = 0;
	gg->e[4] = 0;
	gg->e[5] = 0;
	
	gg->d[0] = 3;
	gg->d[1] = 1;
	gg->d[2] = 1;
	gg->d[3] = 1;
	return gg;
}


sparsegraph* genGraph3(){
	sparsegraph* gg = new sparsegraph();
	gg->nv = 3;
	gg->nde = 4;
	SG_ALLOC(*gg, gg->nv, gg->nde, "ALLOC FAILED");
	
	gg->v[0] = 0;
	gg->v[1] = 1;
	gg->v[2] = 3;
	
	gg->e[0] = 1;
	gg->e[1] = 0;
	gg->e[2] = 2;
	gg->e[3] = 1;
	
	gg->d[0] = 1;
	gg->d[1] = 2;
	gg->d[2] = 1;
	return gg;
}



int main(){
	
	//BEGIN TESTING

	sparsegraph* g = genGraph1();
	sparsegraph* h = genGraph3();
	float points[5];
	Configuration* gg = new Configuration(1, points);
	Configuration* hh = new Configuration(1, points);
	gg->addGraph(g);
	hh->addGraph(h);
	gg->canonizeGraph();
	hh->canonizeGraph();
	std::cout<< gg->compareGraph(hh)<<std::endl;
	delete gg;
	delete hh;
	SG_FREE(*g);
	delete g;
	std::cout<<"ALL GOOD!"<<std::endl;
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
