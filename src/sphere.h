#ifndef _SPHERE_H_
#define _SPHERE_H_
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

//THIS HAS BEEN BORROWED FROM http://stackoverflow.com/questions/5988686/creating-a-3d-sphere-in-opengl-using-visual-c
class Sphere
{
	
	protected:
	std::vector<GLfloat> vertices;
//	std::vector<GLfloat> normals;
//	std::vector<GLfloat> texcoords;
	std::vector<GLushort> indices;
	
	public:
	static GLint modelLoc;
	static glm::mat4 configTransform;
	
	Sphere(float radius, unsigned int rings, unsigned int sectors);
	Sphere(){}
	Sphere& operator=(const Sphere& other);

	void draw(GLfloat x, GLfloat y, GLfloat z);
	
};

#endif

