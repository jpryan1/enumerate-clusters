#ifndef _EDGE_H_
#define _EDGE_H_
#include <vector>
#include <cmath>
#include <iostream>

//This fixes a deprecation issue
#define GLM_FORCE_RADIANS 1

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glew.h"
#include <GLFW/glfw3.h>


class Edge
{
	
	protected:
	std::vector<GLfloat> vertices_normals;

	std::vector<GLushort> indices;
	
	public:
	static GLint modelLoc;
	
	Edge(float radius, unsigned int slices);
	Edge(){}

	void draw(GLfloat x1, GLfloat y1, GLfloat z1,
			  GLfloat x2, GLfloat y2, GLfloat z2);
	
};

#endif

