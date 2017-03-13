#include "Configuration.h"
#include <bitset>
#include <vector>


#define TOLMAX 10*DEL_S
#define TOLMIN DEL_S/8
#define vTol 2*DEL_S
#define DEL_S0 5e-2
#define DEL_S 5e-3
#define NEWTON_TOL 8.8e-16
#define tolA 1e-5
#define MAX_NEWTON_ITERATIONS 1000


Animation Configuration::animation;


template<typename _Matrix_Type_> //This is taken from https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}



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
	//.
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


Configuration Configuration::makeCopy(){
	return Configuration((double*) &(this->p), this->g);
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
	this->v = right_null_space.col(0);
	this->v = this->v/this->v.norm();
	
	
	//Find left nullspace, check size
	MatrixXd rigid_x_T = rigid_x.transpose();
	FullPivLU<MatrixXd> leftlu(rigid_x_T);
	MatrixXd left_null_space = leftlu.kernel();
	int W = left_null_space.cols();
	if(W==1 && left_null_space.isZero(1e-5)){
		if(useNumericalMethod){
			return numerical_findDimension(right_null_space);
		}
		return V;
		
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
			rigid_x = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);
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
			vi=Matrix<double, 3*NUM_OF_SPHERES, 1>::Zero(); //zero out for next iteration
			R_vi = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);

		}
		
	}
	
	
	if(useNumericalMethod){
		return numerical_findDimension(right_null_space);
	}
	
	return V;
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
int Configuration::numerical_findDimension(MatrixXd& right_null_space){
	std::vector<ConfigVector> basis;
	ConfigVector jump, proj, tang, orth;
	for(int i=0; i< right_null_space.cols(); i++){
		
		jump = DEL_S0 * right_null_space.col(i) + this->p;
	
		project(jump, proj);
		if( (proj - jump).norm() <= TOLMAX){
			tang = proj - this->p;
			if( tang.norm() < TOLMIN){
				continue;
			}
			orth = tang;
			for(int j=0; j<basis.size();j++){
				orth = orth - orth.dot(basis[j])*basis[j];
			}
			if(orth.norm()>vTol){
				basis.push_back(orth/orth.norm());
			}
		}
		jump = -DEL_S0 * right_null_space.col(i) + this->p;
		project(jump, proj);
		if( (proj - jump).norm() <= TOLMAX){
			tang = proj - this->p;
			if( tang.norm() < TOLMIN){
				continue;
			}
			orth = tang;
			for(int j=0; j<basis.size();j++){
				orth = orth - orth.dot(basis[j])*basis[j];
			}
			
			if(orth.norm()>vTol){
				basis.push_back(orth/orth.norm());
			}
		}
	}
	if(basis.size()>0){
		this->v = basis[0];
		this->v = this->v/this->v.norm();
		
	}
	return basis.size();
}

void Configuration::project(ConfigVector& old, ConfigVector& proj){ // TODO include constraint that projection is orthogonal-ish
	
	//this function takes the vector OLD and solves newtons method on the constraint equations to find a zero.
	ConfigVector initial = old; //this makes a copy
	double jump_size, F_size;
	//The following is a repeat of what's already been done, in optimization this MUST BE DONE AWAY WITH.
	int numOfContacts = 0;
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES;j++){
			if(this->hasEdge(i,j)){
				numOfContacts++;
			}
		}
	}
	//Now allocate
	MatrixXd rigid_x;
	
	MatrixXd jacob_inv(3*NUM_OF_SPHERES, numOfContacts+6);
	MatrixXd F_vec(numOfContacts+6, 1);
	int iterations = 0;
	do{
		iterations++;
		rigid_x = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES); //zero it out
		populateRigidityMatrix( rigid_x, initial); //repopulate it wrt new initial vector
		rigid_x *=2; //now its the jacobian
		jacob_inv = pseudoInverse(rigid_x);//(rigid_x.transpose()*rigid_x).inverse()*rigid_x;
		populate_F_vec(initial, F_vec);
		//F_vec is constraints evaluated on initial
		
		proj = initial - jacob_inv * F_vec;
		
		jump_size = (initial-proj).norm();
		F_size = F_vec.norm();
		initial = proj;
		
	}while(jump_size>NEWTON_TOL && F_size>NEWTON_TOL);
//cap number of iters
	//ensure F_vec actually is small at destination
	
	
}

void Configuration::populate_F_vec(ConfigVector& initial, MatrixXd& F_vec){
	F_vec = MatrixXd::Zero(F_vec.rows(), 1);
	int k = 0;
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if(this->hasEdge(i,j)){
				for(int m =0; m<3; m++){
					F_vec(k) += pow(initial(3*i + m) - initial(3*j + m), 2);
				}
				F_vec(k) -= 1;
				k++;
			}
		}
	}
}


std::vector<Configuration> Configuration::walk(){
	std::vector<Configuration> newConfigs;
	ConfigVector next, proj;
	Configuration firststep;
	ConfigVector direction = this->v;
	MatrixXd rigid_x;
	std::vector<Contact> contacts;
	
	double p, q;
	
	for(int i=0; i<2; i++){
		
		direction *=-1;
		next = DEL_S*direction + this->p;
		project(next,proj);
		
		firststep = Configuration((double* )&this->p, (graph*) &this->g);
		if(firststep.dimensionOfTangentSpace()!=1){
			continue;
		}
		contacts = checkForNewContacts(proj);
		if(contacts.size()>0){
			continue;
		}
		int numOfContacts = 0;
		for(int i=0; i<NUM_OF_SPHERES-1; i++){
			for(int j=i+1; j<NUM_OF_SPHERES;j++){
				if(this->hasEdge(i,j)){
					numOfContacts++;
				}
			}
		}
		
		//Now populate
		rigid_x = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);
	
		populateRigidityMatrix( rigid_x, proj);
		
		//Find right nullspace
		FullPivLU<MatrixXd> rightlu(rigid_x);
		
		MatrixXd right_null_space = rightlu.kernel();
		
		p = right_null_space.rows();
		q = right_null_space.cols();
		right_null_space = right_null_space.householderQr().householderQ();
		right_null_space = right_null_space.block(0,0,p,q);
		
		direction = right_null_space*right_null_space.transpose()*direction;
		direction = direction/direction.norm();
		//project?
	//	direction = project direction onto nullspace
		std::cout<<"Entering while loop"<<std::endl;
		double duration;
		while(1){
			next = DEL_S*direction + proj;
			//std::cout<<"Beginning projection..."<<std::endl;
			project(next, proj);
			if(1){
				animation.setP(proj);
			}
			//std::cout<<"Ended projection!"<<std::endl;
			contacts = checkForNewContacts(proj);
			if(contacts.size()>1){
				std::cout<<"MANY CONTACTS!"<<std::endl;
			}
			else if(contacts.size()==1){
				//we found a new config. Make copy and add it.
				Configuration newC((double*) &proj,  (graph*) &this->g);
				newC.addEdge(contacts[0].first, contacts[0].second);
				newConfigs.push_back(newC);
				std::cout<<"BREAKING WHILE LOOP!!"<<std::endl;
				break;
			}
			
			
			
			//The below is just the process of updating the direction wrt the projected vector
			//Now populate
			rigid_x = MatrixXd::Zero(numOfContacts+6, 3*NUM_OF_SPHERES);
			
			populateRigidityMatrix( rigid_x, proj);
			
			//Find right nullspace
			FullPivLU<MatrixXd> rightlu(rigid_x);
			MatrixXd right_null_space = rightlu.kernel();
			p = right_null_space.rows();
			q = right_null_space.cols();
			right_null_space = right_null_space.householderQr().householderQ();
			right_null_space = right_null_space.block(0,0,p,q);
			
		
			direction = right_null_space*right_null_space.transpose()*direction;
			//this is dangerous
			direction = direction/direction.norm();

		}
	}
	
	
	/*For each direction, we alternate between taking a step of size ∆s along the manifold in the tangent direction vk, and projecting back onto the manifold. After each projection we form the rigidity matrix in (S2), compute its null space V, and find the next tangent direction vk+1 by the least-squares projection of vk onto V. This ensures that we keep going in the same direction, i.e. we don’t accidentally start moving backwards along the manifold, and it provides an estimate of the single tangent direction when the path is singular.
	After the first step, we check the dimension as in section 2.1, and stop moving if this dimension has increased or decreased. For n = 13 it increased for 3851 paths and decreased for 23. We do not check the dimension after the first step, as this is very time-consuming.
	*/
	std::cout<<"Walk is ended!"<<std::endl;
	return newConfigs;
	
	
}
std::vector<Contact> Configuration::checkForNewContacts(ConfigVector proj){
	std::vector<Contact> newContacts;
	double dist;
	for(int i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
//			if(this->hasEdge(i,j)){
//				continue;
//			}
			dist = 0;
			for(int k=0; k<3; k++){
				dist += pow(proj(3*i+k) - proj(3*j+k) , 2);
			}
			if(dist-1<=tolA){
				
				if(this->hasEdge(i,j)){
					continue;
				}
				newContacts.push_back(Contact(i,j));
			}else{
				if(this->hasEdge(i,j)){
					std::cout<<"BROKEN CONFIGURATION!!"<<std::endl;
				}
			}
		}
	}
	return newContacts;
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


ConfigVector Configuration::getP(){
	return this->p;
}

graph* Configuration::getG(){
	return this->g;
}
