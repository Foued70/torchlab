#include <QtGui/QMouseEvent>
#include "ScanWidget.h"
#include "opengl.h"

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

ScanWidget::ScanWidget(QWidget *parent) : QGLWidget(parent) {
  setMouseTracking(false);
  camera_x = 4; camera_y = 4; camera_z = 4;
  
}

ScanWidget::~ScanWidget() {
  vertexBuffer->destroy();
  polyBuffer->destroy();
}

void ScanWidget::initializeGL() {
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0, 0, 0, 0);

    vertexBuffer = new QGLBuffer(QGLBuffer::VertexBuffer);
    vertexBuffer->create();
    bool bound = vertexBuffer->bind();
    printf("vbuf %d %d\n", bound, vertexBuffer->bufferId());
    vertexBuffer->setUsagePattern(QGLBuffer::StaticDraw);

    const GLsizeiptr vertex_size = NUMBER_OF_CUBE_VERTICES*NUMBER_OF_CUBE_COMPONENTS_PER_VERTEX*sizeof(GLfloat);
    const GLsizeiptr color_size = NUMBER_OF_CUBE_COLORS*NUMBER_OF_CUBE_COMPONENTS_PER_COLOR*sizeof(GLubyte);

    vertexBuffer->allocate(vertex_size + color_size);
    GLvoid* buf = vertexBuffer->map(QGLBuffer::ReadWrite);
    printf("buf %d %d %d\n", buf, vertex_size, color_size);
    memcpy(buf, s_cubeVertices, vertex_size);
    buf += vertex_size;
    memcpy(buf, s_cubeColors, color_size);
    vertexBuffer->unmap();

    // Describe to OpenGL where the vertex data is in the buffer
    glVertexPointer(3, GL_FLOAT, 0, (GLvoid*)((char*)NULL));

    // Describe to OpenGL where the color data is in the buffer
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, (GLvoid*)((char*)NULL+vertex_size));

    polyBuffer = new QGLBuffer(QGLBuffer::VertexBuffer);
    polyBuffer->create();
    bound = polyBuffer->bind();
    printf("ibuf %d %d\n", bound, polyBuffer->bufferId());
    polyBuffer->setUsagePattern(QGLBuffer::StaticDraw);
    polyBuffer->allocate(s_cubeIndices, NUMBER_OF_CUBE_INDICES*sizeof(GLubyte));


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
    glClear(GL_COLOR_BUFFER_BIT);

    camera();
    // glBegin(GL_LINE_LOOP);
    // glColor3f(1,0,0); glVertex3f(-0.5, -0.5, -0.5);
    // glColor3f(1,1,0); glVertex3f(-0.5, -0.5,  0.5);
    // glColor3f(0,1,0); glVertex3f( 0.5, -0.5,  0.5);
    // glColor3f(0,0,1); glVertex3f( 0.5, -0.5, -0.5);
    // glColor3f(1,0,0); glVertex3f( 0.5,  0.5, -0.5);
    // glColor3f(1,1,0); glVertex3f( 0.5,  0.5,  0.5);
    // glColor3f(0,1,0); glVertex3f(-0.5,  0.5,  0.5);
    // glColor3f(0,0,1); glVertex3f(-0.5,  0.5, -0.5);
    // glEnd();
    
    // DO NOT use polyBuffer->bind().  It binds to the wrong place (GL_ARRAY_BUFFER).
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, polyBuffer->bufferId());
    glEnableClientState(GL_COLOR_ARRAY);
    // printf("%d\n", glGetError());

    vertexBuffer->bind();
    glEnableClientState(GL_VERTEX_ARRAY);
    
    GLint id;
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &id);
    // printf("vbuf_id %d\n", id);
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &id);
    // printf("ibuf_id %d %d\n", polyBuffer->bufferId(), id);
    // This is the actual draw command
    glDrawElements(GL_QUADS, NUMBER_OF_CUBE_INDICES, GL_UNSIGNED_BYTE, (GLvoid*)((char*)NULL));

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

