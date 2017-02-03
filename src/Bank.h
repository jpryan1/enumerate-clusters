
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
	Bank(){
		root = new BankNode();
	}
	
	~Bank(){
		delete root;
	}
	
	int add(Configuration* c){ //RETURNS 1 IF ADDED, 0 IF ALREADY IN BANK
		if(root->configs.size()<1){
			root->configs.push_back(c);
			return 1;
		}
		return recursiveAdd(c, root);
	}
	
	
private:
	
	BankNode* root;
	int recursiveAdd(Configuration* c, BankNode* node){
		int comp = c->compareGraph(node->configs[0]);
		if(comp == 0){
			if(/*point vecs are equal*/){
				return 0;
			}else{
				node->configs.push_back(c);
				return 1;
			}
		}else if(comp < 0){
			if(!node->left){
				node->left = new BankNode();
				node->left->configs.push_back(c);
				return 1;
			}else{
				return recursiveAdd(c, node->left);
			}
		}else{
			if(!node->right){
				node->right = new BankNode();
				node->right->configs.push_back(c);
				return 1;
			}else{
				return recursiveAdd(c, node->right);
			}

		}
		
	}

};



#endif
