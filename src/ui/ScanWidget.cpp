#include "opengl.h"
#include "Core3_2_context.h"
#include <QtGui/QMouseEvent>
#include "ScanWidget.h"
#include <QDebug>
#include <QKeyEvent>
#include <QFile>
#include <QString>

#include <iostream>
#include <string>

// vertex coords array
static GLfloat s_cubeVertices[] =
{
	-1.0, +1.0, +1.0,  -1.0, -1.0, +1.0,  +1.0, +1.0, +1.0,  +1.0, -1.0, +1.0,        // v0-v1-v2-v3
 	+1.0, +1.0, +1.0,  +1.0, -1.0, +1.0,  +1.0, +1.0, -1.0,  +1.0, -1.0, -1.0,          // v2-v3-v4-v5
	+1.0, +1.0, -1.0,  +1.0, -1.0, -1.0,  -1.0, +1.0, -1.0,  -1.0, -1.0, -1.0,    // v4-v5-v6-v7
	-1.0, +1.0, -1.0,  -1.0, -1.0, -1.0,  -1.0, +1.0, +1.0,  -1.0, -1.0, +1.0     // v6-v7-v0-v1
};
#define NUMBER_OF_CUBE_VERTICES 16
#define NUMBER_OF_CUBE_COMPONENTS_PER_VERTEX 3

// color array
GLubyte s_cubeColors[] =
{
	// Bleh. I hate unsigned bytes for colors. Normalized floats are much more elegant.
	255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255,
	0,255,0,255, 0,255,0,255, 0,255,0,255, 0,255,0,255,
	0,0,255,255, 0,0,255,255, 0,0,255,255, 0,0,255,255,
	0,255,255,255, 0,255,255,255, 0,255,255,255, 0,255,255,255,

};
#define NUMBER_OF_CUBE_COLORS 16
#define NUMBER_OF_CUBE_COMPONENTS_PER_COLOR 4

// Describes a box, but without a top and bottom
GLubyte s_cubeIndices[] =
{
	0,1,2,3,
	4,5,6,7,
	8,9,10,11,
 	12,13,14,15
};
#define NUMBER_OF_CUBE_INDICES 16

ScanWidget::ScanWidget(const QGLFormat& format, QWidget* parent ) : QGLWidget( new Core3_2_context(format), parent ),
      vertexBuffer( QGLBuffer::VertexBuffer ),
      polyBuffer(QGLBuffer::VertexBuffer)
{
  setMouseTracking(false);
  camera_x = 4; camera_y = 4; camera_z = 4;

}

ScanWidget::~ScanWidget() {
}



GLuint ScanWidget::prepShaderProgram( const QString& vertexShaderPath,
                                     const QString& fragmentShaderPath )
{
    struct Shader {
        const QString&  filename;
        GLenum       type;
        GLchar*      source;
    }  shaders[2] = {
        { vertexShaderPath, GL_VERTEX_SHADER, NULL },
        { fragmentShaderPath, GL_FRAGMENT_SHADER, NULL }
    };

    GLuint program = glCreateProgram();

    for ( int i = 0; i < 2; ++i ) {
        Shader& s = shaders[i];
        QFile file( s.filename );
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
            qWarning() << "Cannot open file " << s.filename;
            exit( EXIT_FAILURE );
        }
        QByteArray data = file.readAll();
        file.close();
        s.source = data.data();

        if ( shaders[i].source == NULL ) {
            qWarning() << "Failed to read " << s.filename;
            exit( EXIT_FAILURE );
        }
        GLuint shader = glCreateShader( s.type );
        glShaderSource( shader, 1, (const GLchar**) &s.source, NULL );
        glCompileShader( shader );

        GLint  compiled;
        glGetShaderiv( shader, GL_COMPILE_STATUS, &compiled );
        if ( !compiled ) {
            qWarning() << s.filename << " failed to compile:" ;
            GLint  logSize;
            glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &logSize );
            char* logMsg = new char[logSize];
            glGetShaderInfoLog( shader, logSize, NULL, logMsg );
            qWarning() << logMsg;
            delete [] logMsg;

            exit( EXIT_FAILURE );
        }

        glAttachShader( program, shader );
    }

    /* Link output */
    glBindFragDataLocation(program, 0, "fragColor");

    /* link  and error check */
    glLinkProgram(program);

    GLint  linked;
    glGetProgramiv( program, GL_LINK_STATUS, &linked );
    if ( !linked ) {
        qWarning() << "Shader program failed to link";
        GLint  logSize;
        glGetProgramiv( program, GL_INFO_LOG_LENGTH, &logSize);
        char* logMsg = new char[logSize];
        glGetProgramInfoLog( program, logSize, NULL, logMsg );
        qWarning() << logMsg ;
        delete [] logMsg;

        exit( EXIT_FAILURE );
    }

    /* use program object */
    glUseProgram(program);

    return program;
}

void ScanWidget::initializeGL() {
  printf("OpenGL %s GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

  // Set the clear color to black
  glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
 
  // Prepare a complete shader program...
  m_shader = prepShaderProgram( ":/simple.vert", ":/simple.frag" );
 
  // We need us some vertex data. Start simple with a triangle ;-)
  float points[] = { -0.5f, -0.5f, 0.0f, 1.0f,
                      0.5f, -0.5f, 0.0f, 1.0f,
                      0.0f,  0.5f, 0.0f, 1.0f };
                      
  
  glGenVertexArrays(1, &m_vertexBuffer);
  glBindVertexArray(m_vertexBuffer);
  GLuint  vertexBuffer;
  glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, 3 * 6 * sizeof(float), points, GL_STATIC_DRAW);
  GLuint positionAttribute = glGetAttribLocation(m_shader, "vertex");
  GLuint colorAttribute = glGetAttribLocation(m_shader, "color");
  glEnableVertexAttribArray(positionAttribute);
  glVertexAttribPointer(positionAttribute, 3, GL_FLOAT, GL_FALSE, sizeof(float)*6, (const GLvoid *)0);
  glEnableVertexAttribArray(colorAttribute);
  glVertexAttribPointer(colorAttribute, 3, GL_FLOAT, GL_FALSE, sizeof(float)*6, (const GLvoid *)(sizeof(float)*3));
      
}

void ScanWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, float(w)/float(h), 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void ScanWidget::paintGL() {
    // Clear the buffer with the current clearing color
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // camera();

    // Draw stuff
    glDrawArrays( GL_TRIANGLES, 0, 3 );
    // glCheckError();
}

void ScanWidget::camera() {
  float focal_x = 0, focal_y = 0, focal_z = 0;
  float dir_x = focal_x - camera_x;
  float dir_y = focal_y - camera_y;
  float dir_z = focal_z - camera_z;
  float up_x = - dir_x * dir_z;
  float up_y = - dir_y * dir_z;
  float up_z = dir_x * dir_x + dir_y * dir_y;
  // printf("dir %f, %f, %f\n", dir_x, dir_y, dir_z);
  // printf(" up %f, %f, %f\n", up_x, up_y, up_z);
  glLoadIdentity();
  gluLookAt(camera_x, camera_y, camera_z, focal_x, focal_y, focal_z, up_x, up_y, up_z);
}

void ScanWidget::mousePressEvent(QMouseEvent *event) {

}
void ScanWidget::mouseMoveEvent(QMouseEvent *event) {
    // printf("%d, %d\n", event->x(), event->y());
}

void ScanWidget::keyPressEvent(QKeyEvent* event) {
  // printf("%d\n", event->key());
  switch(event->key()) {
    case Qt::Key_Escape:
      // close();
      break;
    case Qt::Key_Left:
      camera_x -= 0.1;
      break;
    case Qt::Key_Right:
      camera_x += 0.1;
      break;
    case Qt::Key_Up:
      camera_z += 0.1;
      break;
    case Qt::Key_Down:
      camera_z -= 0.1;
      break;
    default:
      event->ignore();
      break;
  }

  updateGL();
}

