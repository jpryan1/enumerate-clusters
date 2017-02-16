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

int Configuration::matches(Configuration* other){
	//graphs have already been checked, just check points.
	
	return 1;
	
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
	graph canonized[NUM_OF_SPHERES];
//	
	densenauty(this->g, lab, ptn, orbits, &options, &stats, 1, NUM_OF_SPHERES, canonized);
//
	
	double newPoints[3*NUM_OF_SPHERES];
	
	for(int i=0; i< NUM_OF_SPHERES; i++){
		for(int j=0; j<3; j++){
			newPoints[3*i+j] = (this->p) (3*lab[i]+j);
			
		}
	}

	memcpy(&(this->p), newPoints, sizeof(double)*NUM_OF_SPHERES*3);
	memcpy(this->g, canonized, sizeof(graph)*NUM_OF_SPHERES);
}


Configuration* Configuration::makeCopy(){
	return new Configuration((double*) &(this->p), this->g);
}


void Configuration::deleteEdge(int a, int b){
	DELELEMENT(g+a, b);
	DELELEMENT(g+b, a);
}

void Configuration::addEdge(int a, int b){
	ADDONEEDGE(g, a, b, 1);
}

int Configuration::hasEdge(int i, int j){
	return ((g[i] & bit[j])>0);
}

int Configuration::dimensionOfTangentSpace(bool useNumericalMethod = true){
	
	//construct rigidity matrix
	//rigidity matrix has dimension numContacts x 3*numofspheres
	//first we calculate dimensions of matrix
	//- rigidity matrix has dimension numContacts x 3*numofspheres

	int numOfContacts = 0;
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES;j++){
			if(this->hasEdge(i,j)){
				numOfContacts++;
			}
		}
	}
	
	//Now allocate
	MatrixXd rigid_x = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);
	
	//Now populate
	populateRigidityMatrix( rigid_x, this->p);
	
	
	//Find right nullspace, check size
	FullPivLU<MatrixXd> rightlu(rigid_x);
	MatrixXd right_null_space = rightlu.kernel();
	int V = right_null_space.cols();
	if( V==1 && right_null_space.isZero(1e-5)){ //1e-5 is the precision with which to check.
		return 0;
	}
	
	
	//Find left nullspace, check size
	MatrixXd rigid_x_T = rigid_x.transpose();
	FullPivLU<MatrixXd> leftlu(rigid_x_T);
	MatrixXd left_null_space = leftlu.kernel();
	int W = left_null_space.cols();
	if(W==1 && left_null_space.isZero(1e-5)){
		if(useNumericalMethod){
			return numerical_findDimension();
		}
		return V;
		//DEPENDING ON WHETHER TO USE THIS OR NUMERICAL METHOD
	}

	
	//Compute Q matrices, check for sign-definiteness via eigendecomposition
	
	MatrixXd Q = MatrixXd::Zero( V, V);
	MatrixXd R_vi = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);
	ConfigVector vi;
	VectorXd eigs(V);
	bool flag;
	//this can be vectorized better... TODO
	for(int k=0; k< W; k++){
		for(int i=0; i<V; i++){
			vi << right_null_space.col(i);
			populateRigidityMatrix(R_vi, vi);
			for(int j=0; j<V; j++){
				Q(i,j) = left_null_space.col(k).transpose() * R_vi * right_null_space.col(j);
			}
			
			
			//test Q for sign definiteness here.
			SelfAdjointEigenSolver<MatrixXd> es(Q, EigenvaluesOnly);
			eigs = es.eigenvalues();
			
			flag = true;
			if(eigs(0)>0){
				for(int idx = 0; idx<V; idx++){
					if(eigs(idx)<=1e-5){
						flag = false;
					}
				}
				if(flag){
					return 0;
				}
			}else if(eigs(0)<0){
				for(int idx = 0; idx<V; idx++){
					if(eigs(idx)>=-1e-5){
						flag = false;
					}
				}if(flag){
					return 0;
				}
			}
//
			vi=MatrixXd::Zero(NUM_OF_SPHERES*3, 1); //zero out for next iteration
			R_vi = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);

		}
		
	}
	
	
	
	return V; // is this correct?

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
				row++;
			}
		}
	}
	//Certainly, row is now numOfContacts. So, we insert the equations fixing the position of the triangle.
	rigid(row++, 0) = 1;
	rigid(row++, 1) = 1;
	rigid(row++, 2) = 1;
	rigid(row++, 4) = 1;
	rigid(row++, 5) = 1;
	rigid(row++, 8) = 1;
	
	
	
}
int Configuration::numerical_findDimension(){
	
	
	return 0;
}



int Configuration::walk(){
	
	//TODO
	
	
	/*For each direction, we alternate between taking a step of size ∆s along the manifold in the tangent direction vk, and projecting back onto the manifold. After each projection we form the rigidity matrix in (S2), compute its null space V, and find the next tangent direction vk+1 by the least-squares projection of vk onto V. This ensures that we keep going in the same direction, i.e. we don’t accidentally start moving backwards along the manifold, and it provides an estimate of the single tangent direction when the path is singular.
	After the first step, we check the dimension as in section 2.1, and stop moving if this dimension has increased or decreased. For n = 13 it increased for 3851 paths and decreased for 23. We do not check the dimension after the first step, as this is very time-consuming.
	*/
	return 1;
	
	
}


void Configuration::printDetails(){
	std::cout<<"\nPrinting config details"<<std::endl;
	graph* g1 =  this->g;
	std::cout<<"Graph:"<<std::endl;
	for(int i=0; i<NUM_OF_SPHERES; i++){
			std::cout << std::bitset<sizeof(graph)*8>(g1[i]) << std::endl;
	}
	std::cout<<"Points:"<<std::endl;
	for(int i=0; i<NUM_OF_SPHERES*3; i+=3){
		for(int j=0; j<3; j++){
			std::cout<<(this->p)(i+j)<<" ";
		}std::cout<<std::endl;
	}
	std::cout<<"Done printing config details\n"<<std::endl;
	
}

