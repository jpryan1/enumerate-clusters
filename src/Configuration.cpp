#include "Configuration.h"



int Configuration::compareGraph(Configuration* other){
	//ASSUMPTION IS THAT SPARSEGRAPHS ARE IN CANONICAL FORM
	//WORSTCASE RUNTIME - NUMBER OF EDGES
	sparsegraph* a = this->graph;
	sparsegraph* b = other->graph;
	if(a->nv > b->nv){
		return 1;
	}else if(a->nv < b->nv){
		return -1;
	}
	if(a->nde > b->nde){
		return 1;
	}else if(a->nde < b->nde){
		return -1;
	}
	int edges_a = 0;
	int edges_b = 0;
	
	int* ad = a->d;
	int* ae = a->e;
	int* bd = b->d;
	int* be = b->e;
	size_t* av = a->v;
	size_t* bv = b->v;
	
	for(int i=0; i<a->nv; i++){
		if(ad[i] > bd[i]){
			return 1;
		}else if(ad[1] < bd[1]){
			return -1;
		}
		
		for(int j=0; j < ad[i]; j++){
			edges_a |= (1<<ae[av[i]+j]);
			edges_b |= (1<<be[bv[i]+j]);
		}
		
		if(edges_a > edges_b){
			return 1;
		}else if(edges_a < edges_b){
			return -1;
		}
		edges_a = 0;
		edges_b = 0;
	}
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
