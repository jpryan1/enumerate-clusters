#include "Configuration.h"

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
		if(!project(next, proj)){
			continue;
		}
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
		
		bool temp = false;
		MatrixXd right_null_space = getRightNullSpace(rigid_x, &temp);
		if(temp) std::cout<<"Something bad happened!!"<<std::endl;
		//Get orthonormalized right nullspace (the Q in QR factorization)
		p = right_null_space.rows();
		q = right_null_space.cols();
		right_null_space = right_null_space.householderQr().householderQ();
		MatrixXd leftcols = right_null_space.leftCols(q);
		right_null_space = leftcols;
		
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
			if(!project(next, proj)){
				break;
			}
			//Update animation
			if(animation){
				animation->setP(proj);
			}
			//Check if we have any new contacts
			contacts = checkForNewContacts(proj, small_step);
			
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
				newC.prewalk_points = this->p;
				memcpy(newC.prewalk_graph, this->g, NUM_OF_SPHERES*sizeof(graph));
				for(int j=0; j<contacts.size(); j++){
					newC.addEdge(contacts[j].first, contacts[j].second);
				}
				newConfigs.push_back(newC);
				break;
			}
			
			
			//The below is the projection of the direction vector onto the right nullspace
			// as described above
			
			rigid_x = MatrixXd::Zero(this->num_of_contacts+6, 3*NUM_OF_SPHERES);
			populateRigidityMatrix( rigid_x, proj);
			

			right_null_space = getRightNullSpace(rigid_x, &temp);
			if(temp) std::cout<<"Something bad happened1!!"<<std::endl;
			
			
			p = right_null_space.rows();
			q = right_null_space.cols();
			right_null_space = right_null_space.householderQr().householderQ();
			leftcols = right_null_space.leftCols(q);
			right_null_space = leftcols;
			direction = right_null_space*right_null_space.transpose()*direction;
			direction = direction/direction.norm();
			
		}
	}
	
	std::vector<Configuration> projectedConfigs;
	for(int i=0; i<newConfigs.size(); i++){
		bool add = true;
		while(1){
			
			if(!newConfigs[i].project()){
				add = false;
				break;
			}
			contacts = newConfigs[i].checkForNewContacts(newConfigs[i].p, true);
			if(contacts.size() == 0) break;
			for(int j=0; j<contacts.size(); j++){
				newConfigs[i].addEdge(contacts[j].first, contacts[j].second);
			}
		}
		
		if(add) projectedConfigs.push_back(newConfigs[i]);
//		MatrixXd F_vec(newConfigs[i].num_of_contacts+6, 1);
//		newConfigs[i].populate_F_vec(newConfigs[i].p, F_vec);
//		std::cout<<"in walk, projd is "<<F_vec.norm()<<std::endl;
		
	}
	
	
	return projectedConfigs;
	
	
}
