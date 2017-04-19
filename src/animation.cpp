
#include "animation.h"

const GLuint WIDTH = 800, HEIGHT = 600;

// Shaders
const GLchar* vertexShaderSource =
"#version 330 core\n"
"layout (location = 0) in vec3 position;\n"
"layout (location = 1) in vec3 Normal;\n"
"out vec3 normal;\n"
"out vec3 fragPos;\n"
"uniform mat4 model;\n"
"uniform mat4 view;\n"
"uniform mat4 projection;\n"
"void main()\n"
"{\n"
	"fragPos = vec3(model * vec4(position, 1.0));\n"
	"gl_Position = projection*view*vec4(fragPos,1.0f);\n"
	"normal =  mat3(transpose(inverse(model))) * Normal;  ;\n"
"}\0";

const GLchar* fragmentShaderSource = "#version 330 core\n"
"in vec3 normal;\n"
"in vec3 fragPos;\n"
"out vec4 color;\n"
"uniform vec4 ourColor;\n"
"void main()\n"
"{\n"
"	vec3 lightPos = vec3(0, 10, 10);"
"	vec3 ambient = vec3( 0.7, 0.7, 0.7);\n"
"// Diffuse\n"
"	vec3 lightDir = normalize(lightPos - fragPos);\n"
"	float diff = max(dot(normal, lightDir), 0.0);\n"
"	vec3 diffuse = vec3(diff, diff, diff);\n"
"	vec3 result = (ambient + diffuse) * vec3(ourColor);\n"
"	color = vec4(result, 1.0f);\n"
"}\n\0";

void Animation::initialize(){
	//Initialize GLFW
	if (!glfwInit())
	return;
	shouldQuit = false;
	//Set GLFW settings
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	
	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(WIDTH, HEIGHT, "Window", NULL, NULL);
	if (!window)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	
	/* Make the window's context current */
	glfwMakeContextCurrent(window);
	
	// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	glewInit();
	
	
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	glEnable(GL_DEPTH_TEST);
}

//The following function can be simplified with the introduction of a shader.cpp file
void Animation::compileShaders(){
	// Build and compile our shader program
	// Vertex shader
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);
	
	// Check for compile time errors
	GLint success;
	GLchar infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}
	// Fragment shader
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);
	// Check for compile time errors
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}
	// Link shaders
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	// Check for linking errors
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	
	//Use the newly linked shader program
	glUseProgram(shaderProgram);
}



void Animation::setup(){
	initialize();
	compileShaders();
	generateBuffers();
	generateShapes();
	setProjectionMatrices();
}

//REDO the comments here, they're all wrong
void Animation::generateBuffers(){
	//	We use three buffers.
	//	VBO (vertex buffer object) contains the vertex positions
	//	EBO (element buffer object) contains the indices of the vertices (aka the order to draw triangles)
	//	VAO (vertex attribute object) contains the alignment info of the data in the VBO
	glGenVertexArrays(1, &s_VAO);
	glGenVertexArrays(1, &e_VAO);
	glGenBuffers(1, &s_VBO);
	glGenBuffers(1, &s_EBO);
	glGenBuffers(1, &e_VBO);
	glGenBuffers(1, &e_EBO);
	
	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(s_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_EBO);
	

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind VAO
	glBindVertexArray(0);
	
	
	glBindVertexArray(e_VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, e_VBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, e_EBO);

	
	
	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);
	
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind VAO
	glBindVertexArray(0);
	
	
	
}

void Animation::generateShapes(){
	//Bind VAO so sphere data can be transferred to VBO buffers on construction
	glBindVertexArray(s_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
	
	sphere = Sphere(0.2, 12, 24);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind
	glBindVertexArray(0);
	
	
	glBindVertexArray(e_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, e_VBO);
	
	edge = Edge(0.05, 12);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind
	glBindVertexArray(0);
	
}

void Animation::setProjectionMatrices(){
	//This function creates 4x4 transformation matrices which are then passed to uniforms.
	//The uniforms are accessed by the shaderProgram, which then applies the matrices to the vertex positions
	
	//Initialize to identity
	glm::mat4 view;
	glm::mat4 projection;
	
	//View is at z = +8
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -6.0f));
//	view = glm::rotate(view, (float) -(M_PI/8.0), glm::vec3(0.0f, 1.0f, 0.0f));
	//Projection has 45 degree FoV, aspect ratio given by the window's, and
	//records only those objects within 0.1f and 100.0f of the "camera"
	projection = glm::perspective(glm::radians(45.0f), (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 100.0f);
	
	// Get the locations of the shader's uniforms
	modelLoc = glGetUniformLocation(shaderProgram, "model");
	viewLoc = glGetUniformLocation(shaderProgram, "view");
	GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
	colorLoc = glGetUniformLocation(shaderProgram, "ourColor");
	//Give the location of the model uniform to the Sphere class, so that translations can be applied
	//when Sphere::draw is called.
	Sphere::modelLoc = modelLoc;
	Edge::modelLoc = modelLoc;
	// Set the uniforms of the other matrices now - they won't change.
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
	
}

void Animation::draw(){
	//This loop runs until the window is closed. When this happens, this function reaches the last line, which results in the
	//entire program ending
	GLfloat radius, camX, camZ;
	radius = 6.0f;
	while (!glfwWindowShouldClose(window) && !shouldQuit)
	{
		/* Poll for and process events */
		glfwPollEvents();

		//Set background color
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Draw!
//		camX = sin(glfwGetTime()) * radius;
//		camZ = cos(glfwGetTime()) * radius;
//		glm::mat4 view;
//		view = glm::lookAt(glm::vec3(camX, 0, camZ), glm::vec3(0.0, 0, 0.0), glm::vec3(0.0, 1.0, 0.0));
//		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
		drawShapes();
		
		/* Swap front and back buffers */
		glfwSwapBuffers(window);
	}
	
	//Cleanup
	glDeleteVertexArrays(1, &e_VAO);
	glDeleteVertexArrays(1, &s_VAO);
	glDeleteBuffers(1, &s_VBO);
	glDeleteBuffers(1, &s_EBO);
	glDeleteBuffers(1, &e_VBO);
	glDeleteBuffers(1, &e_EBO);
	glfwTerminate();
	
}



void Animation::drawShapes(){
	p_lock.lock();
	
	
	glBindVertexArray(e_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, e_VBO);
	
	
	glUniform4f(colorLoc, 0.2, 0.2, 0.2, 1.0f);

	g_lock.lock();
	
	
	int i;
	for(i=0; i<NUM_OF_SPHERES-1; i++){
		for(int j=i+1; j<NUM_OF_SPHERES; j++){
			if((this->g[i] & bit[j])>0){
				edge.draw(p(3*i), p(3*i+1), p(3*i+2), p(3*j), p(3*j+1), p(3*j+2));
				
			}
		}
	}
	g_lock.unlock();
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	
	
	glUniform4f(colorLoc, 87.0/255.0, 6.0/255.0, 140.0/255.0, 1.0f);
	glBindVertexArray(s_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
	for(i=0; i<NUM_OF_SPHERES; i++){
		sphere.draw(p(3*i), p(3*i+1), p(3*i+2));
	}
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	
	p_lock.unlock();
}


void Animation::quit(){
	shouldQuit = true;
}


void Animation::setP(ConfigVector update){
	p_lock.lock();
	p = update;
	p_lock.unlock();
}

void Animation::setG(graph* g){
	g_lock.lock();
	memcpy(this->g, g, sizeof(graph)*NUM_OF_SPHERES);
	g_lock.unlock();
}






