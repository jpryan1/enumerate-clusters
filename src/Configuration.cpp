#include "Configuration.h"

Animation* Configuration::animation;


Configuration::Configuration(double* points, graph* adj, bool setTriangle){
	
	memcpy(&p, points, 3*NUM_OF_SPHERES*sizeof(double));
	
	memcpy(g, adj, NUM_OF_SPHERES*sizeof(graph));
	
	this->num_of_contacts = 0;
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES;j++){
			if(this->hasEdge(i,j)){
				this->num_of_contacts++;
			}
		}
	}
	if(setTriangle){
		chooseTriangle();
		fixTriangle();
	}
	
}
void Configuration::checkTriangle(int a){
	if(!(hasEdge(triangle[0],triangle[1])&&hasEdge(triangle[1],triangle[2])&&hasEdge(triangle[0],triangle[2]))){
		std::cout<<"broken triangle "<<a<<std::endl;
	}
}

void Configuration::show(int a){
	animation->setP(this->p);
	animation->setG(this->g);
	double t;
	t = time(0);
	while(time(0)-t<a);
}


Configuration Configuration::makeCopy(bool setTriangle){
	return Configuration((double*) &(this->p), this->g, setTriangle);
}


void Configuration::deleteEdge(int a, int b){
	DELELEMENT(g+a, b);
	DELELEMENT(g+b, a);
	this->num_of_contacts--;
}

void Configuration::addEdge(int a, int b){
	ADDONEEDGE(g, a, b, 1);
	this->num_of_contacts++;
}

int Configuration::hasEdge(int i, int j){
	return ((g[i] & bit[j])>0 || (g[j] & bit[i])>0);
}

//TODO the following doesn't account for whether the fixed points are collinear D:
void Configuration::populateRigidityMatrix(MatrixXd& rigid, ConfigVector& x){
	int row = 0;
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if(this->hasEdge(i,j)){
				//ith trio should be p_i - p_j, jth trio should be p_j-p_i
				for(int k=0; k<3; k++){
					rigid(row, 3*i+k) = x(3*i+k) - x(3*j+k);
					rigid(row, 3*j+k) = x(3*j+k) - x(3*i+k);
				}
				//std::cout<<row<<std::endl;
				row++;
			}
		}
	}
	//Certainly, row is now numOfContacts. So, we insert the equations fixing the position of the triangle.
	//std::cout<<"YO "<<row<<" "<<rigid.rows()<<std::endl;
	//std::cout<<triangle[0]<<std::endl;
	rigid(row++, 3*triangle[0]) = 1;
	rigid(row++, 3*triangle[0]+1) = 1;
	rigid(row++, 3*triangle[0]+2) = 1;
	//std::cout<<"YO "<<row<<std::endl;
	
	rigid(row++, 3*triangle[1]+1) = 1;
	rigid(row++, 3*triangle[1]+2) = 1;
	rigid(row++, 3*triangle[2]+2) = 1;
	
}std::vector<Contact> Configuration::checkForNewContacts(ConfigVector proj, bool smallTol){
	std::vector<Contact> newContacts;
	double dist, tol;
	if(smallTol){
		tol = tolA*0.01;
	}else{
		tol = tolA;
	}
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
//			This is commented out for now. It would save time, but right now we
//			leave in a check for broken configurations
//			if(this->hasEdge(i,j)){
//				continue;
//			}
			
			dist = 0;
			for(int k=0; k<3; k++){
				dist += pow(proj(3*i+k) - proj(3*j+k) , 2);
			}
			if(dist-1<=tol){
				
				if(this->hasEdge(i,j)){
					continue;
				}
				newContacts.push_back(Contact(i,j));
			}else{
//				if(this->hasEdge(i,j)){
//					std::cout<<"BROKEN CONFIGURATION!!"<<std::endl;
//				}
			}
		}
	}
	return newContacts;
}


void Configuration::printDetails(){
//	if(animation){
//		animation->setP(this->p);
//		animation->setG(this->g);
//		double t;
//		t = time(0);
//		while(time(0)-t<5);
//	}
	//std::cout<<"\nPrinting config details"<<std::endl;
	graph* g1 =  this->g;
	//std::cout<<num_of_contacts<<" contacts"<<std::endl;
	std::cout<<"Graph:"<<std::endl;
	for(int i=0; i<NUM_OF_SPHERES; i++){
		std::cout << std::bitset<8*sizeof(graph)>(g1[i]);// << std::endl;
	}std::cout<<std::endl;
	std::cout<<"Points:"<<std::endl;
	for(int i=0; i<NUM_OF_SPHERES*3; i+=3){
		for(int j=0; j<3; j++){
			std::cout<<(this->p)(i+j)<<" ";
		}
	}
	std::cout<<std::endl;
	//std::cout<<"Done printing config details\n"<<std::endl;
	
}


ConfigVector Configuration::getP(){
	return this->p;
}

graph* Configuration::getG(){
	return this->g;
}
