
#define MAXN WORDSIZE
#define ONE_WORD_SETS 1
//the above is specially used since our graphs will have small order, see nauty documentation

#include <iostream>
#include <queue>
#include "Configuration.h"
#include "Bank.h"


//TODO Consider fixed size vectorization (16 byte alignment) via Eigen

int main(){
	
	//BEGIN TESTING
		
	return 0;
	//END TESTING
	
	//INITIALIZATION
	//std::queue<Configuration*> Queue;
	//Bank* bank = new Bank();
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
