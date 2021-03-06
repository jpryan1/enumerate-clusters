#include "Configuration.h"

extern Timer timer;

MatrixXd Configuration::getRightNullSpace(MatrixXd rigid, bool* null_flag){
	timer.start();
	if(this->isRegular){
		FullPivLU<MatrixXd> rightlu(rigid);
		MatrixXd right_null_space = rightlu.kernel();
		if( right_null_space.cols()==1 && right_null_space.isZero(1e-5)){ //1e-5 is the precision with which to check.
			*null_flag = true;
		}
		timer.end(0);

		return right_null_space;
	}
	JacobiSVD<MatrixXd> svd(rigid, ComputeFullU | ComputeFullV);
	MatrixXd svdv = svd.matrixV();
	MatrixXd sing = svd.singularValues();
	int smallsing = 0;
	for(int i=0; i<sing.size(); i++){
		if(sing(i)<1e-6){
			//std::cout<<"Small singular value"<<std::endl;
			smallsing = sing.size()-i;
		}
	}
	int V = smallsing + svdv.cols() - sing.size();
	//std::cout<<smallsing<<" + "<<svdv.cols() <<" - "<<sing.size()<<" = "<<V<<std::endl;
	if(V==0) {
		*null_flag = true;
	}
	timer.end(0);
	return svdv.rightCols(V);//rightlu.kernel();
	

}
int Configuration::dimensionOfTangentSpace(bool useNumericalMethod){
	
	
	//construct rigidity matrix
	//rigidity matrix has dimension numContacts x 3*numofspheres
	//first we calculate dimensions of matrix
	//- rigidity matrix has dimension numContacts x 3*numofspheres
	
	
	//Now allocate
	MatrixXd rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
	
	//Now populate
	populateRigidityMatrix( rigid_x, this->p);
	
	
	//first, assume it isn't regular, then update later
	this->isRegular = false;
	
	//Find right null space
	bool null_flag = false;
	MatrixXd right_null_space = getRightNullSpace(rigid_x, &null_flag);
	if(null_flag) return 0;
	int V = right_null_space.cols();
	
	this->v = right_null_space.col(0);
	this->v = this->v/this->v.norm();
	
	if(V <= 3*NUM_OF_SPHERES -6-num_of_contacts){
		this->isRegular = true;
		return V;
	}else{
		//std::cout<<"Irregular..."<<std::endl;
		this->isRegular = false;
	}
	
	
	//Find left null space
	null_flag = false;
	MatrixXd left_null_space = getRightNullSpace(rigid_x.transpose(), &null_flag);
	int W;
	if(null_flag){
		W = 0;
	}else{
		W = left_null_space.cols();
	}
	if(W == 0){
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


int Configuration::numerical_findDimension(MatrixXd& right_null_space){
	
	std::vector<ConfigVector> basis;
	ConfigVector jump, proj, tang, orth;
	for(int i=0; i< right_null_space.cols(); i++){
		
		jump = DEL_S0 * right_null_space.col(i) + this->p;
		
		if(project(jump, proj)){
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
		jump = -DEL_S0 * right_null_space.col(i) + this->p;
		if(project(jump, proj)){
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
	}
	if(basis.size()>0){
		this->v = basis[0];
		this->v = this->v/this->v.norm();
		
	}
	return basis.size();
}
