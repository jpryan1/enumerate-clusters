
#ifndef  _BANK_H_    /* only process this file once */
#define  _BANK_H_


#include "Configuration.h"
#include <vector>

struct BankNode{
	std::vector<Configuration*> configs;
	BankNode* left;
	BankNode* right;
};




class Bank{

	//This will have to act like a binary tree of configurations, where the ordering is adj matrix in canonical form
	//TODO make this a self-balancing tree
	
public:
	Bank(Configuration* c){
		root = new BankNode();
		root->configs.push_back(c);
	}
	
	~Bank(){
		//TODO
	}
	
//	int add(Configuration* c){ //RETURNS 1 IF ADDED, 0 IF ALREADY IN BANK
//		return recursiveAdd(c, root);
//	}
	
	
private:
	
	BankNode* root;

	
//	int recursiveAdd(Configuration* c, BankNode* node){
//		int comp = c->graphCompare(node->configs[0]->graph);
//		
//		
//	}

};



#endif
