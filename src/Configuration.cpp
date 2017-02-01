#include "Configuration.h"



int Configuration::compareTo(Configuration* other){
	//ASSUMPTION IS THAT SPARSEGRAPHS ARE IN CANONICAL FORM
	
	
	
	return 0;
}

void Configuration::canonizeGraph(){
	
	
	//These arrays are passed to sparsenauty in canonization, so that function can write its data somewhere
	//TODO edit sparsenauty to not record data into these
	//long story short - sparsenauty does more things than just canonize, which we don't care about
	//sure, this allocation is done every time this function is called,
	//but I think the work is done at compiletime, not runtime
	//besides, the alternative is passing pointers to arrays to this method, which is ugly.
	int lab[NUM_OF_SPHERES];
	int ptn[NUM_OF_SPHERES];
	int orbits[NUM_OF_SPHERES];
	
	DEFAULTOPTIONS_SPARSEGRAPH(options);
	options.getcanon = true;
	
	statsblk stats;
	sparsegraph* canonized = new sparsegraph();
	//create the object
	
	//here's the main line of this function
	//this function creates the arrays for canonized
	sparsenauty(graph,lab, ptn, orbits, &options, &stats, canonized);
	
	
	SG_FREE(*graph); //free up the arrays
	delete graph;	//free up the object
	graph = canonized;
	
	
	
	
	
}
