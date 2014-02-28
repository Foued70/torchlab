// Link statically with GLEW
#define GLEW_STATIC

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

int main()
{
	//bool close=false;
	glewExperimental = GL_TRUE;
	glfwInit();

	GLuint vertexBuffer;
	glGenBuffers(1, &vertexBuffer);
	printf("%u\n", vertexBuffer);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL", NULL, NULL); // Windowed
//	GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL", glfwGetPrimaryMonitor(), NULL); // Fullscreen
	glfwMakeContextCurrent(window);
	glfwSetWindowShouldClose(window, GL_FALSE);
	while(!glfwWindowShouldClose(window))
	{
	    glfwSwapBuffers(window);
	    glfwPollEvents();
	    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	    	glfwSetWindowShouldClose(window, GL_TRUE);
	}
	glfwTerminate();
	return 0;
}