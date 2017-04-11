#include "Configuration.h"


#define TOLMAX 10*DEL_S
#define TOLMIN DEL_S/8
#define vTol 2*DEL_S
#define DEL_S0 5e-2
#define DEL_S 5e-3
#define NEWTON_TOL 1e-15
#define tolA 1e-3
//new tolerance for jump back 1e-3
#define tolD 1e-6
//tolD should be like 1e-6 ... this may be because we don't baby step yet
#define MAX_NEWTON_ITERATIONS 1000


Animation* Configuration::animation;


template<typename _Matrix_Type_> //This is taken from https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


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

int Configuration::compareGraph(Configuration& other){
	if(this->num_of_contacts!=other.num_of_contacts){
		if(this->num_of_contacts>other.num_of_contacts) return 1;
		return -1;
	}//bug possible
	
	
	
//	if(animation&&num_of_contacts<24){
//		
//		std::cout<<"Showing first one"<<std::endl;
//		
//		animation->setP(this->p);
//		animation->setG(this->g);
//		double t;
//		t = time(0);
//		while(time(0)-t<9);
////		std::cout<<"Showing second one"<<std::endl;
////		animation->setP(other.p);
////		animation->setG(other.g);
////		t = time(0);
////		while(time(0)-t<5);
//	}

	setword* g1 = (setword*) this->g;
	setword* g2 = (setword*) other.g;


	for(int i=0; i<NUM_OF_SPHERES; i++){
		if(g1[i]<g2[i]){
			return -1;
		}else if(g1[i]>g2[i]){
			return 1;
		}
	}return 0;
}

int Configuration::matchesHelper(Configuration& other){
	ConfigVector copy = other.p;
	ConfigVector diff = copy - this->p;
	double temp = diff.norm();
	if(diff.norm()<tolD) return 1;
	for(int i=2; i <3*NUM_OF_SPHERES; i+=3) copy(i)  = -copy(i);
	diff = copy - this->p;
	if(diff.norm()<tolD) return 1;
	return 0;
}
int Configuration::matches(Configuration& other){
	//graphs have already been checked, just check points.
	
	int tri[3];
	memcpy(tri, triangle, 3*sizeof(int));
	
	int newtri[3];
	
	for(int i=0; i<3; i++) newtri[i] = tri[i];
	setTriangle(newtri);
	fixTriangle();
	if(matchesHelper(other)) return 1;
	
	for(int i=0; i<3; i++) newtri[(i+1)%3] = tri[i];
	setTriangle(newtri);
	fixTriangle();
	if(matchesHelper(other)) return 1;
	
	for(int i=0; i<3; i++) newtri[(i+2)%3] = tri[i];
	setTriangle(newtri);
	fixTriangle();
	if(matchesHelper(other)) return 1;
	
	for(int i=0; i<3; i++) newtri[i] = tri[2-i];
	setTriangle(newtri);
	fixTriangle();
	if(matchesHelper(other)) return 1;
	
	for(int i=0; i<3; i++) newtri[(i+1)%3] = tri[2-i];
	setTriangle(newtri);
	fixTriangle();
	if(matchesHelper(other)) return 1;
	
	for(int i=0; i<3; i++) newtri[(i+2)%3] = tri[2-i];
	setTriangle(newtri);
	fixTriangle();
	if(matchesHelper(other)) return 1;
	
	return 0;
	
	setTriangle(tri);
	fixTriangle();
//	if(animation&&num_of_contacts<24){
//		
//		std::cout<<"Showing first one"<<std::endl;
//		animation->setP(this->p);
//		animation->setG(this->g);
//		double t;
//		t = time(0);
//		while(time(0)-t<5);
//		std::cout<<"Showing second one"<<std::endl;
//		animation->setP(other.p);
//		animation->setG(other.g);
//		t = time(0);
//		while(time(0)-t<5);
//	}

	
	
	return 0;
	
}



void Configuration::chooseTriangle(){
	
	std::vector<int> degreeList[NUM_OF_SPHERES];
	
	for(int i=0; i<NUM_OF_SPHERES; i++){
		int numEdges = 0;
		graph temp = this->g[i];
		while(temp>0){
			if(temp%2==1){
				numEdges++;
			}
			temp = temp>>1;
		}
		degreeList[numEdges].push_back(i);
	}
	
	
	
	//so now the ith slot in degreeList is a vector containing the vertices with degree i
	
	int SBD[NUM_OF_SPHERES];
	//short for SORTED BY DEGREE
	
	int idx = NUM_OF_SPHERES-1;
	for(int i=NUM_OF_SPHERES-1; i>=0; i--){
		for(int j = 0; j<degreeList[i].size(); j++){
			SBD[idx--] = degreeList[i][j];
		}
	}
	
	//now sorted by degree has the vertices sorted by degree
	
	
	//note that the below forloop could take NUM_OF_SPHERES^3 time, but
	//should usually take much less. We are looking among high degree vertices
	//for a triangle.
	for(int i=NUM_OF_SPHERES-1; i>=0; i--){
		for(int j = i-1; j>=0; j--){
			if(this->hasEdge(SBD[i],SBD[j])){
				for(int k=j-1; k>=0; k--){
					if(this->hasEdge(SBD[i],SBD[k])&&this->hasEdge(SBD[j],SBD[k])){
						triangle[0] = SBD[i];
						triangle[1] = SBD[j];
						triangle[2] = SBD[k];
						return;
					}
				}
			}
		}
	}
	std::cout<<"Error: no triangle found."<<std::endl;
	exit(0);
}


void Configuration::fixTriangle(){
	

	double translate[3];
	for(int i=0; i<3; i++){
		translate[i] = -this->p(3*triangle[0]+i);
	}
	for(int i=0; i<NUM_OF_SPHERES; i++){
		for(int j=0; j<3; j++){
			this->p(3*i+j) += translate[j];
		}
	}
	
	
	double spherepos1[3];
	for(int i=0; i<3; i++){
		spherepos1[i] = this->p(3*triangle[1]+i);
	}
	Vector3d rotate1_axis;
	//take dot with x axis
	//cos for angle
	
	Vector3d temp;
	
	
	double rotate1_ang = acos(spherepos1[0]);
	if(rotate1_ang>1e-8){
		//cross with x axis for rot axis, normalize
		rotate1_axis << 0, spherepos1[2], -spherepos1[1];
		rotate1_axis.normalize();
		//rotate by negative that angle
		AngleAxisd rotationMatrix1( rotate1_ang, rotate1_axis);
		for(int i=0; i<NUM_OF_SPHERES; i++){
			for(int j=0; j<3; j++){
				temp(j) = this->p(3*i+j);
			}
			temp = rotationMatrix1*temp;
			for(int j=0; j<3; j++){
				this->p(3*i+j) = temp(j);
			}
		}
	}
	
	
	
	
	double spherepos2[3];
	for(int i=0; i<3; i++){
		spherepos2[i] = this->p(3*triangle[2]+i);
	}
	

	double rotate2_ang = acos(spherepos2[1]/(sqrt(   pow(spherepos2[1],2) + pow(spherepos2[2],2)     )));
	if(spherepos2[2]<0){
		rotate2_ang *=-1;
	}
	
	if(std::abs(rotate2_ang)>1e-8){
		AngleAxisd rotationMatrix2;
		rotationMatrix2 = AngleAxisd(-rotate2_ang,  Vector3d::UnitX());
		for(int i=0; i<NUM_OF_SPHERES; i++){
			for(int j=0; j<3; j++){
				temp(j) = this->p(3*i+j);
			}
			temp = rotationMatrix2*temp;
			for(int j=0; j<3; j++){
				this->p(3*i+j) = temp(j);
			}
		}
		
		
	}
	
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
	chooseTriangle();
	fixTriangle();
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

int Configuration::dimensionOfTangentSpace(bool useNumericalMethod = true){
	
	
	//construct rigidity matrix
	//rigidity matrix has dimension numContacts x 3*numofspheres
	//first we calculate dimensions of matrix
	//- rigidity matrix has dimension numContacts x 3*numofspheres


	//Now allocate
	MatrixXd rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
	
	//Now populate
	populateRigidityMatrix( rigid_x, this->p);
	
	
	//Find right nullspace, check size
	FullPivLU<MatrixXd> rightlu(rigid_x);
	
	
	//SVD for precision - 1e-6 - 1e-7
	
	MatrixXd right_null_space = rightlu.kernel();
	
	
	
	int V = right_null_space.cols();
	if( V==1 && right_null_space.isZero(1e-5)){ //1e-5 is the precision with which to check.
		
		return 0;
	}
	if(V == 3*NUM_OF_SPHERES -6-num_of_contacts){
		this->isRegular = true;
	//	return V; //ASK MIRANDA IF THIS IS OKAY
	}else{
	//	std::cout<<"Irregular..."<<std::endl;
		this->isRegular = false;
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
	MatrixXd R_vi = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
	ConfigVector vi;
	VectorXd eigs(V);
	bool flag;
	//this can be vectorized better... TODO
	for(int k=0; k< W; k++){
		for(int i=0; i<V; i++){
			vi << right_null_space.col(i);
			rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
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
			R_vi = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);

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
void Configuration::project(){
	project(this->p, this->p);
}
void Configuration::project(ConfigVector& old, ConfigVector& proj){ // TODO include constraint that projection is orthogonal-ish
	
	//this function takes the vector OLD and solves newtons method on the constraint equations to find a zero.
	ConfigVector initial = old; //this makes a copy
	double jump_size, F_size;

	
	//deflation technique - lookup
	
	//Now allocate
	MatrixXd rigid_x;
	
	MatrixXd jacob_inv(3*NUM_OF_SPHERES, this->num_of_contacts+6);
	MatrixXd F_vec(this->num_of_contacts+6, 1);
	int iterations = 0;
	//std::cout<<"HERE"<<std::endl;
	do{
		iterations++;
	//	std::cout<<(this->num_of_contacts+6)<<std::endl;
		rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES); //zero it out
	//	std::cout<<"HERE1"<<std::endl;
		
		populateRigidityMatrix( rigid_x, initial); //repopulate it wrt new initial vector
	//	std::cout<<"HERE2"<<std::endl;
		
		rigid_x *=2; //now its the jacobian
		populate_F_vec(initial, F_vec);
	//	std::cout<<"HERE3"<<std::endl;
		
		if(this->isRegular){
			
			proj =  initial - rigid_x.fullPivLu().solve(F_vec);
			
			
		}
		else{
			
			jacob_inv = pseudoInverse(rigid_x);//(rigid_x.transpose()*rigid_x).inverse()*rigid_x;
			//F_vec is constraints evaluated on initial
		
			proj = initial - jacob_inv * F_vec;
		}
		
		jump_size = (initial-proj).norm();
		F_size = F_vec.norm();
		initial = proj;
		
	}while(jump_size>NEWTON_TOL && F_size>NEWTON_TOL && iterations<500);
	if(iterations==500){
		std::cout<<"Did not converge... "<<std::endl;
	}
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
	MatrixXd rigid_x;
	std::vector<Contact> contacts;
	double p, q, t;
	//We take steps in both directions along the 1D manifold (hence the two-step for-loop)
	ConfigVector direction = this->v;
	ConfigVector copy_d = direction;
	for(int i=0; i<2; i++){
		direction = copy_d*pow(-1,i);
		
		//Take a teensy step in that direction...
		next = DEL_S0*direction + this->p;
		
		//...and project back onto the manifold
		project(next, proj);
		
		//We create a Configuration corresponding to that first step
		// to check its tangent space dimension
		firststep = Configuration((double* )&this->p, (graph*) &this->g);
		
		if(firststep.dimensionOfTangentSpace()!=1){
			
			//Any gain or loss in dimension, and we ditch this direction
			continue;
			
		}
		
		contacts = checkForNewContacts(proj);
		if(contacts.size()>0){
			//If taking that step resulted in a new contact, don't walk in that direction
			continue;
		}
		
		
		//Now we're committed to walking along the manifold in that direction
		
		//Now populate
		rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
		populateRigidityMatrix( rigid_x, proj);
		
		//Find right nullspace
		FullPivLU<MatrixXd> rightlu(rigid_x);
		MatrixXd right_null_space = rightlu.kernel();
		
		//Get orthonormalized right nullspace (the Q in QR factorization)
		p = right_null_space.rows();
		q = right_null_space.cols();
		right_null_space = right_null_space.householderQr().householderQ();
		right_null_space = right_null_space.block(0,0,p,q);
		
		//Project direction vector onto right nullspace
		direction = right_null_space*right_null_space.transpose()*direction;
		direction = direction/direction.norm();
		if(animation){
			animation->setG(g);
		}
		double step_size = DEL_S0;
		bool small_step = false;
		while(1){
			//Take a step...
			next = step_size*direction + proj;
			//...and project back onto manifold
			ConfigVector p_before = proj;
			project(next, proj);
			//Update animation
			if(animation){
				animation->setP(proj);
			}
			//Check if we have any new contacts
			contacts = checkForNewContacts(proj);
			
			if(contacts.size()>=1){
				if(!small_step){
					step_size = DEL_S;
					small_step = true;
					proj = p_before;
					continue;
				}
				//We reached the end of our walk!
				//Add the new configuration to our newConfigs list.
				Configuration newC((double*) &proj,  (graph*) &this->g);
				for(int j=0; j<contacts.size(); j++){
					newC.addEdge(contacts[j].first, contacts[j].second);
					newConfigs.push_back(newC);
				}
				break;
			}
			
			
			//The below is the projection of the direction vector onto the right nullspace
			// as described above
			
			rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
			populateRigidityMatrix( rigid_x, proj);
			rightlu = FullPivLU<MatrixXd>(rigid_x);
			right_null_space = rightlu.kernel();
			p = right_null_space.rows();
			q = right_null_space.cols();
			right_null_space = right_null_space.householderQr().householderQ();
			right_null_space = right_null_space.block(0,0,p,q);
			direction = right_null_space*right_null_space.transpose()*direction;
			direction = direction/direction.norm();

		}
	}
	
	
	for(int i=0; i<newConfigs.size(); i++){
		newConfigs[i].setTriangle(triangle);
	//	std::cout<<newConfigs[i].num_of_contacts;
		newConfigs[i].project();
		
		contacts = newConfigs[i].checkForNewContacts(newConfigs[i].p, true);
		for(int j=0; j<contacts.size(); j++){
			newConfigs[i].addEdge(contacts[j].first, contacts[j].second);
		}
		
		//We have to do this again because we projected, so the triangle may have moved a bit.
		newConfigs[i].canonize();
	}
	

	return newConfigs;
	
	
}
std::vector<Contact> Configuration::checkForNewContacts(ConfigVector proj, bool smallTol){
	std::vector<Contact> newContacts;
	double dist, tol;
	if(smallTol){
		tol = tolD;
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
	//graph* g1 =  this->g;
	//std::cout<<num_of_contacts<<" contacts"<<std::endl;
	//std::cout<<"Graph:"<<std::endl;
//	for(int i=0; i<NUM_OF_SPHERES; i++){
//		std::cout << std::bitset<NUM_OF_SPHERES>(g1[i]);// << std::endl;
//	}std::cout<<std::endl;
//	std::cout<<"Points:"<<std::endl;
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
