in vec4 sVertex;
in vec3 sNormal;
in vec2 sTexCoords;

uniform uint selectedVertexCount;
uniform vec3 selectedVertexPosition;
out float vertexSelected; //Can't pass bools around. 0.0 = false, 1.0 = true

out vec3 vertexCenterScreenSpace;

bool vectors_equal(vec3 a, vec3 b) {
  float epsilon = 0.001;
  if( abs(b.x-a.x) < epsilon && 
      abs(b.y-a.y) < epsilon &&
      abs(b.z-a.z) < epsilon )
    return true;

  return false;
}

void main() {
  if (selectedVertexCount > uint(0)) {
    if (vectors_equal(sVertex.xyz, selectedVertexPosition))
      vertexSelected = 1.0;
    else
      vertexSelected = 0.0;
  }
  else
    vertexSelected = 0.0;

  gl_Position = sModelViewProjectionMatrix * sVertex;
  vertexCenterScreenSpace.xy = gl_Position.xy / gl_Position.w;
  vertexCenterScreenSpace.z = ((gl_Position.z/gl_Position.w)+1.0) / 2.0;
}
