#include "Configuration.h"
#include <bitset>



int Configuration::compareGraph(Configuration* other){
	
	setword* g1 = (setword*) this->g;
	setword* g2 = (setword*) other->g;
	
	for(int i=0; i<NUM_OF_SPHERES; i++){
		if(g1[i]<g2[i]){
			return -1;
		}else if(g1[i]>g2[i]){
			return 1;
		}
	}return 0;
}

void Configuration::canonize(){
	
	
	//These arrays are passed to sparsenauty in canonization, so that function can write its data somewhere
	//TODO edit nauty to not record data into these
	//long story short - nauty does more things than just canonize, which we don't care about
	//sure, this allocation is done every time this function is called,
	//but I think the work is done at compiletime, not runtime
	//besides, the alternative is passing pointers to arrays to this method, which is ugly.
	int lab[NUM_OF_SPHERES];
	int ptn[NUM_OF_SPHERES];
	int orbits[NUM_OF_SPHERES];
	
	DEFAULTOPTIONS_GRAPH(options);
	options.getcanon = true;
	
	statsblk stats;
//	
	graph *canonized = (graph*) malloc(sizeof(graph)*NUM_OF_SPHERES);
//	
	densenauty(this->g, lab, ptn, orbits, &options, &stats, 1, NUM_OF_SPHERES, canonized);
//	
	//create the object
	
	//here's the main line of this function
	//this function creates the arrays for canonized
	
	float newPoints[3*NUM_OF_SPHERES];
	
	for(int i=0; i< NUM_OF_SPHERES; i++){
		for(int j=0; j<3; j++){
			newPoints[3*i+j] = (*this->p) (3*lab[i]+j);
			
		}
	}

	memcpy(this->p, newPoints, sizeof(float)*NUM_OF_SPHERES*3);
	free(this->g);
	this->g = canonized;
	
	
	
}

void Configuration::deleteEdge(int a, int b){
	DELELEMENT(g+a, b);
	DELELEMENT(g+b, a);
}

void Configuration::addEdge(int a, int b){
	ADDONEEDGE(g, a, b, 1);
}



void Configuration::printDetails(){
	std::cout<<"\nPrinting config details"<<std::endl;
	setword* g1 = (setword*) this->g;
	for(int i=0; i<NUM_OF_SPHERES; i++){
		std::cout << std::bitset<64>(g1[i]) << std::endl;
	}
	std::cout<<"Done printing config details\n"<<std::endl;
}

