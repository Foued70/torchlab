
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

extern "C"
{
#include "TH.h"
}


// GLM for maths ... forcing radians because degrees are lame
#define GLM_FORCE_RADIANS 
#include <vector>
#include <glm/glm.hpp>

/*
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>
*/

// GLFW for window management ... define forces glfw to use gl3 header
#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>


using namespace std;

extern "C"
{

// TODO: write custom versions of these, they are just for testing.
typedef struct {
    GLuint vertex;
    GLuint fragment;
} shaders_t;

char* loadFile(const char *fname, GLint &fSize)
{
    ifstream::pos_type size;
    char * memblock;
    std::string text;

    // file read based on example in cplusplus.com tutorial
    ifstream file (fname, ios::in|ios::binary|ios::ate);
    if (file.is_open())
    {
        size = file.tellg();
        fSize = (GLuint) size;
        memblock = new char [size];
        file.seekg (0, ios::beg);
        file.read (memblock, size);
        file.close();
        cout << "file " << fname << " loaded" << endl;
        text.assign(memblock);
    }
    else
    {
        cout << "Unable to open file " << fname << endl;
        exit(1);
    }
    return memblock;
}

// printShaderInfoLog
// From OpenGL Shading Language 3rd Edition, p215-216
// Display (hopefully) useful error messages if shader fails to compile
void printShaderInfoLog(GLint shader)
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;

    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);

    // should additionally check for OpenGL errors here

    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetShaderInfoLog(shader,infoLogLen, &charsWritten, infoLog);
        cout << "InfoLog:" << endl << infoLog << endl;
        delete [] infoLog;
    }

    // should additionally check for OpenGL errors here
}

void printLinkInfoLog(GLint prog) 
{
    int infoLogLen = 0;
    int charsWritten = 0;
    GLchar *infoLog;

    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &infoLogLen);

    // should additionally check for OpenGL errors here

    if (infoLogLen > 0)
    {
        infoLog = new GLchar[infoLogLen];
        // error check for fail to allocate memory omitted
        glGetProgramInfoLog(prog,infoLogLen, &charsWritten, infoLog);
        cout << "InfoLog:" << endl << infoLog << endl;
        delete [] infoLog;
    }
}

shaders_t loadShaders(const char * vert_path, const char * frag_path) {
    GLuint f, v;

    char *vs,*fs;

    v = glCreateShader(GL_VERTEX_SHADER);
    f = glCreateShader(GL_FRAGMENT_SHADER); 

    // load shaders & get length of each
    GLint vlen;
    GLint flen;

    vs = loadFile(vert_path,vlen);
    fs = loadFile(frag_path,flen);

    const char * vv = vs;
    const char * ff = fs;

    glShaderSource(v, 1, &vv,&vlen);
    glShaderSource(f, 1, &ff,&flen);

    GLint compiled;

    glCompileShader(v);
    glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        cout << "Vertex shader not compiled." << endl;
        printShaderInfoLog(v);
    } 

    glCompileShader(f);
    glGetShaderiv(f, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        cout << "Fragment shader not compiled." << endl;
        printShaderInfoLog(f);
    } 
    shaders_t out; out.vertex = v; out.fragment = f;

    delete [] vs; // dont forget to free allocated memory
    delete [] fs; // we allocated this in the loadFile function...

    return out;
}

void attachAndLinkProgram( GLuint program, shaders_t shaders) {
    glAttachShader(program, shaders.vertex);
    glAttachShader(program, shaders.fragment);

    glLinkProgram(program);
    GLint linked;
    glGetProgramiv(program,GL_LINK_STATUS, &linked);
    if (!linked) 
    {
        cout << "Program did not link." << endl;
        printLinkInfoLog(program);
    }
}


static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if ( action == GLFW_PRESS && ( key == GLFW_KEY_ESCAPE  || 
         key == 67 && glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) ) ) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}



int draw_rays( THDoubleTensor* xyz_map )
{


    const char *vert_shader = "shaders/hello_world.vert";
    const char *frag_shader = "shaders/hello_world.frag";

    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    // Set up 4.1 context, have to specify that we are using core_profile on osx 
    glfwWindowHint (GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint (GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint (GLFW_OPENGL_PROFILE,    GLFW_OPENGL_CORE_PROFILE);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        fprintf(stderr, "Failed to open GLFW window\n");
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    // Simple key callback for capturing ctrl-c
    glfwSetKeyCallback(window, key_callback);


    // Print current context 
    printf("shader lang: %s\n",glGetString(GL_SHADING_LANGUAGE_VERSION));
    // Load shaders after greating glcontext
    shaders_t shaders = loadShaders( vert_shader, frag_shader );
    // Create attach and link program
    GLuint prog = glCreateProgram();
    attachAndLinkProgram( prog, shaders );  

    // Ray position data 
    unsigned int vertex_array;
    unsigned int vbo_vertices;
    unsigned int vbo_indices;
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    enum {
            POSITION
    };

    // Take all xyz points    
    cout << "1" << endl;
    long xyz_stack = xyz_map->size[0];
    long xyz_height = xyz_map->size[1];
    long xyz_width = xyz_map->size[2];
    cout << "xyz_stack: " << xyz_stack << endl;
    cout << "xyz_height: " << xyz_height << endl;
    cout << "xyz_width: " << xyz_width << endl;

    double *xyz_data =  THDoubleTensor_data( xyz_map );

    float scl = 0.0001;
    // Add all points to vertex array 
    int cnt = 0;
    for ( int i=0; i<xyz_height; i++ ) {
        for ( int j=0; j<xyz_width; j++ ) {
            
            vertices.push_back( glm::vec3(0.f, 0.f, 0.f) );
            //vertices.push_back( glm::vec3(0.f, 0.f, 0.f) );
            
            vertices.push_back( glm::vec3(scl*float(xyz_data[0*xyz_height*xyz_width + i*xyz_width + j]),
                                          scl*float(xyz_data[1*xyz_height*xyz_width + i*xyz_width + j]),
                                          0.f) );
            
                                        //float(data[2*height*width + k*width + l])) );
            indices.push_back(cnt++);
            indices.push_back(cnt++);

        }
    }



    cout << "2" << endl;

    /*
    vertices.push_back( glm::vec3(-0.6f, -0.4f, 0.f) );
    vertices.push_back( glm::vec3(0.6f, -0.4f, 0.f) );
    vertices.push_back( glm::vec3(0.6f, -0.4f, 0.f) );
    vertices.push_back( glm::vec3(0.f, 0.6f, 0.f) );
    vertices.push_back( glm::vec3(0.f, 0.6f, 0.f) );
    vertices.push_back( glm::vec3(-0.6f, -0.4f, 0.f) );
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(3);
    indices.push_back(4);
    indices.push_back(5);
    */

    // Vertex array 
    glGenVertexArrays(1, &(vertex_array));
    glBindVertexArray(vertex_array);

    // Allocate vbos
    glGenBuffers(1, &vbo_vertices);
    glGenBuffers(1, &vbo_indices);

    //vertices
    glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), 
            &vertices[0], GL_STATIC_DRAW);
    glVertexAttribPointer(POSITION, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(POSITION);

    //indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLushort), 
            &indices[0], GL_STATIC_DRAW);

    //Unplug Vertex Array
    glBindVertexArray(0);
    
    /*
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    */

    glClearColor(0.0,0.0,0.0,1.0);
    glDisable(GL_DEPTH_TEST);

    //glEnable(GL_LINE_SMOOTH); makes everything very slow
    //glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    

    float ratio;
    int width, height;

    glfwGetFramebufferSize(window, &width, &height);
    cout << "width: " << width << endl;
    cout << "height: " << height << endl;

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float) height;
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );        

        // Draw all lines
        glUseProgram(prog);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glBindVertexArray(vertex_array);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
        glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT,0);
        
        //Unplug Vertex Array
        glBindVertexArray(0);
    
        /*
        glLoadIdentity();
        glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
        glBegin(GL_TRIANGLES);
        glColor3f(1.f, 0.f, 0.f);
        glVertex3f(-0.6f, -0.4f, 0.f);
        glColor3f(0.f, 1.f, 0.f);
        glVertex3f(0.6f, -0.4f, 0.f);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0.f, 0.6f, 0.f);
        glEnd();
        */
        /* Swap front and back buffers */
        glfwSwapBuffers(window); 

        /* Poll for and process events */
        glfwPollEvents();  
    }
    glfwTerminate();
    return 0;
}



// Just a basic test
int hello_world(void)
{

    const char *vert_shader = "shaders/hello_world.vert";
    const char *frag_shader = "shaders/hello_world.frag";

    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    // Set up 4.1 context, have to specify that we are using core_profile on osx 
    glfwWindowHint (GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint (GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint (GLFW_OPENGL_PROFILE,    GLFW_OPENGL_CORE_PROFILE);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        fprintf(stderr, "Failed to open GLFW window\n");
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    // Simple key callback for capturing ctrl-c
    glfwSetKeyCallback(window, key_callback);


    // Print current context 
    printf("shader lang: %s\n",glGetString(GL_SHADING_LANGUAGE_VERSION));
    // Load shaders after greating glcontext
    shaders_t shaders = loadShaders( vert_shader, frag_shader );
    // Create attach and link program
    GLuint prog = glCreateProgram();
    attachAndLinkProgram( prog, shaders );  

    // Ray position data 

    unsigned int vertex_array;
    unsigned int vbo_vertices;
    unsigned int vbo_indices;
    std::vector<glm::vec3> vertices;
    std::vector<unsigned short> indices;

    enum {
            POSITION
        };

    vertices.push_back( glm::vec3(-0.6f, -0.4f, 0.f) );
    vertices.push_back( glm::vec3(0.6f, -0.4f, 0.f) );
    vertices.push_back( glm::vec3(0.f, 0.6f, 0.f) );
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);

    // Vertex array 
    glGenVertexArrays(1, &(vertex_array));
    glBindVertexArray(vertex_array);

    // Allocate vbos
    glGenBuffers(1, &vbo_vertices);
    glGenBuffers(1, &vbo_indices);

    //vertices
    glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), 
            &vertices[0], GL_STATIC_DRAW);
    glVertexAttribPointer(POSITION, 3, GL_FLOAT, GL_FALSE,0,0);
    glEnableVertexAttribArray(POSITION);

    //indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLushort), 
            &indices[0], GL_STATIC_DRAW);

    //Unplug Vertex Array
    glBindVertexArray(0);
    
    /*
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    */

    glClearColor(0.0,0.0,0.0,1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        float ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float) height;
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        //glMatrixMode(GL_PROJECTION);

        // Draw triangle
        glUseProgram(prog);

        glBindVertexArray(vertex_array);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT,0);
        
        //Unplug Vertex Array
        glBindVertexArray(0);
    
        /*
        glLoadIdentity();
        glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
        glBegin(GL_TRIANGLES);
        glColor3f(1.f, 0.f, 0.f);
        glVertex3f(-0.6f, -0.4f, 0.f);
        glColor3f(0.f, 1.f, 0.f);
        glVertex3f(0.6f, -0.4f, 0.f);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0.f, 0.6f, 0.f);
        glEnd();
        */

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}



} // extern "C"
