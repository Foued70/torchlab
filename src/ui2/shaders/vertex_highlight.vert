in vec4 sVertex;
in vec3 sNormal;
in vec2 sTexCoords;

out vec3 vertexCenterScreenSpace;

void main() {
  gl_Position = sModelViewProjectionMatrix * sVertex;

  vertexCenterScreenSpace.xy = gl_Position.xy / gl_Position.w;
  vertexCenterScreenSpace.z = ((gl_Position.z/gl_Position.w)+1.0) / 2.0;
}
