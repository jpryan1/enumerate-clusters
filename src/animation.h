
#ifndef  _ANIMATION_H_    /* only process this file once */
#define  _ANIMATION_H_

#define NUM_OF_SPHERES 8
#define GLEW_STATIC
#include <iostream>
#include <mutex>
#include "glew.h"
#include <GLFW/glfw3.h>
#include "sphere.h"
#include "Eigen/Dense"

using namespace Eigen;
typedef Matrix<double, 3*NUM_OF_SPHERES, 1> ConfigVector;


// GLEW


class Animation{
	public:
		Animation();
		void setup();
		void generateBuffers();
		void generateSpheres();
		void setProjectionMatrices();
		void draw();
		void drawSpheres();
		void setP(ConfigVector update);
	private:
		Sphere spheres[NUM_OF_SPHERES];
		GLFWwindow* window;
		GLuint VBO, VAO, EBO, shaderProgram;
		int width, height;
		std::mutex mtx;
		ConfigVector p;


};
#endif
