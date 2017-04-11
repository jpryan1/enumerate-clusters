
#include "Bank.h"

int Bank::add(Configuration c){ //RETURNS 1 IF ADDED, 0 IF ALREADY IN BANK
	
	if(root->configs.size()<1){
		_size++;
		root->configs.push_back(c);
		return 1;
	}
	int temp = recursiveAdd(c, root);
	if(temp){
		_size++;
	}return temp;
}

int Bank::size(){
	return _size;
}
int Bank::recursiveAdd(Configuration c, BankNode* node){
	
	int comp = c.compareGraph(node->configs[0]);
	
	if(comp == 0){
		for(int i=0; i<node->configs.size(); i++){
			if(c.matches(node->configs[i])){
				return 0;
			}
		}
		node->configs.push_back(c);
		return 1;
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


void Bank::printDetails(){
	//std::cout<<"Printing bank details..."<<std::endl;
	if(!root){
		//std::cout<<"No root!"<<std::endl;
		return;
	}
	if(root->configs.size()<1){
		//std::cout<<"Empty root!"<<std::endl;
		return;
	}
	//std::cout<<"Starting with root..."<<std::endl;
	recPrint(root);
	//std::cout<<"Done printing bank details!"<<std::endl;
	
}
void Bank::recPrint(BankNode* node){
	
	if(node->configs.size()<1){
		//std::cout<<"This node has no configs!"<<std::endl;
	}else{
		node->configs[0].printDetails();
		//std::cout<<"This node has "<<node->configs.size()<<" configs"<<std::endl;
	}
	if(!node->left){
		//std::cout<<"No left child!"<<std::endl;
	}else{
		//std::cout<<"Going left..."<<std::endl;
		recPrint(node->left);
	}
	if(!node->right){
		//std::cout<<"No right child!"<<std::endl;
	}else{
		//std::cout<<"Going right..."<<std::endl;
		recPrint(node->right);
	}
	//std::cout<<"Going back up..."<<std::endl;
}



