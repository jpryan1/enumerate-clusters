#include "sphere.h"

GLint Sphere::modelLoc;

Sphere::Sphere(float radius, unsigned int rings, unsigned int sectors)
{
	float const R = 1./(float)(rings-1);
	float const S = 1./(float)(sectors-1);
	int r, s;
	
	vertices_normals.resize(rings * sectors * 6);
//			normals.resize(rings * sectors * 3);
//			texcoords.resize(rings * sectors * 2);
	std::vector<GLfloat>::iterator v = vertices_normals.begin();
//			std::vector<GLfloat>::iterator n = normals.begin();
//			std::vector<GLfloat>::iterator t = texcoords.begin();
	for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
		float const y = sin( -M_PI_2 + M_PI * r * R );
		float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
		float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );
//		
//					*t++ = s*S;
//					*t++ = r*R;
		//
		*v++ = x * radius;
		*v++ = y * radius;
		*v++ = z * radius;
		
		*v++ = x;
		*v++ = y;
		*v++ = z;
	}
	
	indices.resize(rings * sectors * 6); // 4);
	std::vector<GLushort>::iterator i = indices.begin();
	for(r = 0; r < rings-1; r++) for(s = 0; s < sectors-1; s++) {
		*i++ = r * sectors + s;
		*i++ = r * sectors + (s+1);
		*i++ = (r+1) * sectors + (s+1);
		*i++ = r * sectors + s;
		*i++ = (r+1) * sectors + (s+1);
		*i++ = (r+1) * sectors + s;
	}
	
	glBufferData(GL_ARRAY_BUFFER, vertices_normals.size()*sizeof(GLfloat), &vertices_normals[0], GL_STATIC_DRAW);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLushort), &indices[0], GL_STATIC_DRAW);
}

void Sphere::draw(GLfloat x, GLfloat y, GLfloat z)
{
	
	
	glm::mat4 transform;
	transform = glm::translate(transform, glm::vec3(x,y,z));
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(transform));
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, (GLvoid*) 0);
	
}




