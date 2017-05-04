
#ifndef  _ANIMATION_H_    /* only process this file once */
#define  _ANIMATION_H_

#define NUM_OF_SPHERES 11
#define GLEW_STATIC
#include <iostream>
#include <mutex>
#include <atomic>
#include "glew.h"
#include <GLFW/glfw3.h>
#include "sphere.h"
#include "edge.h"
#include "Eigen/Dense"
#include "nauty.h"

using namespace Eigen;
typedef Matrix<double, 3*NUM_OF_SPHERES, 1> ConfigVector;

class Animation{
	public:
	
		Animation(){}
		void initialize();
		void setup();
		void compileShaders();
		void generateBuffers();
		void generateShapes();
		void setProjectionMatrices();
		void draw();
		void drawShapes();
		void setP(ConfigVector update);
		void setG(graph* g);
		void quit();
	private:
		GLuint s_VBO, e_VBO, e_VAO, s_VAO, s_EBO, e_EBO, shaderProgram, modelLoc, colorLoc, viewLoc;
		int width, height;
		std::atomic<bool> shouldQuit;
		Sphere sphere;
		Edge edge;
		GLFWwindow* window;
		
		std::mutex p_lock, g_lock;
		ConfigVector p;
		graph g[NUM_OF_SPHERES];

};
#endif
