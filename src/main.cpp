
#define MAXN WORDSIZE
#define ONE_WORD_SETS 1
//the above is especially used since our graphs will have small order, see nauty documentation

#include <iostream>
#include <fstream>
#include <queue>
#include <unistd.h>
#include <thread>
#include <mutex>
#include "Configuration.h"
#include "Bank.h"
#include "animation.h"
#include "Timer.h"
#define NUM_THREADS 8

std::queue<Configuration> Queue;
std::mutex bank_lock;
std::mutex queue_lock;


Configuration initialCluster();
void breakContactsAndAdd(Configuration& current, Bank& predim);
void enumerateClusters(Configuration* initial, Bank* bank);
void debug();
/*	TODO NOW:
 *
 *	Clean up code, compartmentalize
 *	Implement lighting to make animation nicer
 *	Include drawing of rods in framework in animation
 *	Comment code, including animation code
 *	update makefile
 *
 */

//LATER Consider fixed size vectorization (16 byte alignment) via Eigen
//NOTE: nauty "graph" is just unsigned long (bitwise adj matrix)
Timer timer;


int main(int argc, char** argv){
	Eigen::initParallel();
	std::cout.precision(16);
	
	Animation animation;
	
	//Start with one cluster, built by iteratively adding one vertex with three edges to a triangle
	Configuration c = initialCluster();
	Bank* bank = new Bank(true);
//	Configuration* d = (Configuration*) malloc(sizeof(c));;
//	memcpy(d, &c, sizeof(c));
//	std::cout<<d->matches(c)<<std::endl;
//	d->fixTriangle();
//	std::cout<<d->matches(c)<<std::endl;
////	std::cout<<c.matches(*d)<<std::endl;
////	std::cout<<c.matches(*d)<<std::endl;
//	free(d);
//	return 0;
	std::string opts ="\tOptions:\n\t\t(a)nimate\n\t\t(d)ebug\n\t\t(r)un";
	if(argc<2){
		std::cout<<"usage: ./build/enumerate_clusters {option}\n"<<opts<<std::endl;
	}
	char option = *(argv[1]);
	
	if(option=='a'){
		animation.setup();
	
				//Set the initial position data for the animation
		ConfigVector p = c.getP();
		animation.setP(p);
	
		//Set the initial graph data for the animation
		graph* g = c.getG();
		animation.setG(g);
	
		// By giving the configuration class a pointer to the animation object, methods from within
		// that class may pass data to the animation (this is necessary in the walk method)
		Configuration::animation = &animation;
	
		
		
		/*
		 We take a multithreaded approach - the main thread runs the animation, and the second thread
		runs the program which enumerates the clusters.
		 */
		std::thread enumerate(enumerateClusters, &c, bank);
		animation.draw();
		enumerate.join();
	}else if(option=='r'){
		enumerateClusters(&c, bank);
	}else if(option=='d'){
		animation.setup();
		
		//Set the initial position data for the animation
		ConfigVector p = c.getP();
		animation.setP(p);
		
		//Set the initial graph data for the animation
		graph* g = c.getG();
		animation.setG(g);
		
		// By giving the configuration class a pointer to the animation object, methods from within
		// that class may pass data to the animation (this is necessary in the walk method)
		Configuration::animation = &animation;
		std::thread enumerate(debug);
		//animation.draw();
		enumerate.join();	}else{
		std::cout<<"Error: unknown option.\n"<<opts<<std::endl;
	}
	//bank->printDetails();
	return 0;
	
	
}

void threadFunc(Bank* predim, Bank* bank){
	Configuration current;
	int sz =0;
	do{
		queue_lock.lock();
		current = Configuration(Queue.front());
		Queue.pop();
		int added;
		queue_lock.unlock();
		bank_lock.lock();
		if(!current.canonize()){
			bank_lock.unlock();
			continue;
		}
//		std::cout<<"fn"<<std::endl;
//		MatrixXd F_vec(current.num_of_contacts+6, 1);
//		current.populate_F_vec(current.p, F_vec);
//		std::cout<<"canonize said yes to "<<F_vec.norm()<<std::endl;
//		
		added = bank->add(current);
		bank_lock.unlock();
		if(!added){
			continue;
		}
		breakContactsAndAdd(current, *predim);

	}while(Queue.size() > 0);
	
}




void enumerateClusters(Configuration* initial, Bank* bank){
	double t = time(0);
	Queue.push(*initial);
	Configuration current;
//	int hyper = 0;
//	int hyper2 =0;
//	int hypo2 = 0;
//	int hypo = 0;
//	int total = 0;
	Bank predim;
	while(Queue.size() > 0 && Queue.size()<20){
		current = Configuration(Queue.front());
		Queue.pop();
		if(!current.canonize()) continue;
//		std::cout<<"og"<<std::endl;
//		MatrixXd F_vec(current.num_of_contacts+6, 1);
//		current.populate_F_vec(current.p, F_vec);
//		std::cout<<"canonize said yes to "<<F_vec.norm()<<std::endl;
//		

		if(!bank->add(current)){ //returns 0 if already in bank, otherwise adds and returns 1
			continue;
		}
//		total++;
//		if(current.num_of_contacts<3*NUM_OF_SPHERES-7){
//			hypo2++;
//		}
//		else if(current.num_of_contacts<3*NUM_OF_SPHERES-6){
//			hypo++;
//
//		}
//		else if(current.num_of_contacts>3*NUM_OF_SPHERES-5){
//			hyper2++;
//			
//		}else if(current.num_of_contacts>3*NUM_OF_SPHERES-6){
//			hyper++;
//		}
		breakContactsAndAdd(current, predim);
		
	}
	//std::cout<<"Very hypo: "<<hypo2<<" Hypo: "<<hypo<<" Hyper: "<<hyper<<" Very hyper: "<<hyper2<<std::endl;
	std::thread threads[NUM_THREADS];
	for(int i=0; i<NUM_THREADS; i++){
		threads[i] = std::thread(threadFunc, &predim, bank);
	}
	
	for (auto& th : threads) th.join();
	//threadFunc(&predim, bank);
	std::cout<<"Bank is size "<<bank->size()<<std::endl;
	std::cout<<"Aux banks are "<<predim.size()<<std::endl;
	t = time(0) - t;
	std::cout<<"Elapsed time: "<<t<<" seconds."<<std::endl;
	timer.display();
	
	
	//bank->printDetails();
}







Configuration initialCluster(){
	
	double points[NUM_OF_SPHERES*3];
	
	
	std::ifstream clusterFile;
	clusterFile.open ("first_cluster8.txt");
	if (clusterFile.is_open()){
		for(int i=0; i<NUM_OF_SPHERES*3; i++){
			clusterFile>>points[i];
		}
		
		clusterFile.close();
	}else{
		std::cout<<"Failed to open initial cluster file!"<<std::endl;
		return Configuration();
	}

	
	
	graph g[NUM_OF_SPHERES];
	memset(g, 0, NUM_OF_SPHERES*sizeof(graph));
	
	Configuration* c = new Configuration(points, g, false);
	
	
	for(int i=0; i<NUM_OF_SPHERES-3; i++){
		for(int j=1;j<4;j++){
			c->addEdge(i, i+j);
		}
	}
	c->addEdge(NUM_OF_SPHERES-3, NUM_OF_SPHERES-2);
	c->addEdge(NUM_OF_SPHERES-3, NUM_OF_SPHERES-1);
	c->addEdge(NUM_OF_SPHERES-2, NUM_OF_SPHERES-1);
	
	c->canonize();
	return *c;
}

void breakContactsAndAdd(Configuration& current, Bank& predim ){
	
	//TODO include lookup table to reduce redundancy?
	int dim;
	Configuration copy;
	std::vector<Configuration> walkedTo;
	
	//This double for-loop iterates through all edges in the graph of current
	for(int i=0; i<NUM_OF_SPHERES; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if( !current.hasEdge(i,j) ){
				continue;
			}
			// We make a copy, delete an edge of that copy, then either enter that copy into
			// the queue, or delete it, depending on whether it is rigid.
			copy = current.makeCopy(false);
			copy.deleteEdge(i,j);
			bank_lock.lock();
			if(!copy.canonize()){
				bank_lock.unlock();
				continue;
			}
			if(!predim.add(copy)){
				bank_lock.unlock();
				continue;
			}
			bank_lock.unlock();
			dim = copy.dimensionOfTangentSpace(true);
			
			
			if(dim == 0){
				breakContactsAndAdd(copy, predim);
			}
			
			else if(dim == 1){
				walkedTo = copy.walk();
				for(int k=0; k<walkedTo.size(); k++){ //walking can fail!
					int temp =walkedTo[k].dimensionOfTangentSpace(false);
					if(temp>0) continue;
					queue_lock.lock();
					Queue.push(walkedTo[k]);
					queue_lock.unlock();
				}
			}
		}
	}
	
}


void debug(){
	
	
	Configuration first,second;
	std::ifstream debugFile;
	debugFile.open ("debug.txt");
	if (debugFile.is_open()){
		first.readClusterFromFile(debugFile);
		second.readClusterFromFile(debugFile);
		first.orbitMatches(second,0,true);
		second.orbitMatches(first,0,true);
//		first.canonize();
//		second.canonize();
//		
		first.printOrbits();
		second.printOrbits();
//		while(1){
//			first.show(5);
//			second.show(5);
//		}
//
		debugFile.close();
	}else{
		std::cout<<"Failed to open debug file!"<<std::endl;
		
	}
	
	

	
	
	
	
	
	
}




