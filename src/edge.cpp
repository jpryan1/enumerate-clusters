#include "edge.h"

GLint Edge::modelLoc;

Edge::Edge(float radius, unsigned int slices)
{
	
	vertices_normals.resize(slices*12);

	std::vector<GLfloat>::iterator v = vertices_normals.begin();
	float y,z, ang;
	float interval = 360.0f/slices;
	int i;
	for(i=0; i<slices; i++){
		
		ang = i * interval;
		y = sin(glm::radians(ang));
		z = cos(glm::radians(ang));
		
		*v++ = 0;
		*v++ = radius*y;
		*v++ = radius*z;
		
		*v++ = 0;
		*v++ = y;
		*v++ = z;

		*v++ = 1;
		*v++ = radius*y;
		*v++ = radius*z;
		
		*v++ = 1;
		*v++ = y;
		*v++ = z;
		
	}
	indices.resize(6*slices);
	std::vector<GLushort>::iterator ind = indices.begin();
	
	for( i=0; i<2*(slices-1); i+=2 ){
		*ind++ = i;
		*ind++ = i+1;
		*ind++ = i+3;
		*ind++ = i;
		*ind++ = i+2;
		*ind++ = i+3;
	}
	*ind++ = i;
	*ind++ = i+1;
	*ind++ = 1;
	*ind++ = i;
	*ind++ = 0;
	*ind++ = 1;
	
	
	glBufferData(GL_ARRAY_BUFFER, vertices_normals.size()*sizeof(GLfloat), &vertices_normals[0], GL_STATIC_DRAW);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLushort), &indices[0], GL_STATIC_DRAW);
}

void Edge::draw(GLfloat x1, GLfloat y1, GLfloat z1,
				GLfloat x2, GLfloat y2, GLfloat z2)
{
	glm::mat4 transform;
	
	if(x1 == 1 && fabs(y1) <1e-5 &&  fabs(z1) <1e-5 && x2==0 && y2 == 0 && z2 == 0){
		//This basically says when not to move the edge. it's a bit of a hack
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(transform));
		glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, (GLvoid*) 0);
		return;
	}
	transform = glm::translate(transform, glm::vec3(x1,y1,z1));
	
	glm::vec3 xaxis( 1,0,0);
	glm::vec3 initial( x2-x1, y2-y1, z2-z1);
	float rot_angle = acos( glm::dot(initial, xaxis) );
	if( fabs(rot_angle) > 1e-5)
	{
		glm::vec3 rot_axis = glm::normalize( glm::cross( initial,xaxis) );
		transform = glm::rotate(transform, -rot_angle, rot_axis);
	}
	
	
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(transform));
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, (GLvoid*) 0);
}




