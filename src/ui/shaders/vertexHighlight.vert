out vec3 sVaryingNormal;
out vec3 sVaryingLightDir;
out vec3 sEyeVector;
out float sAttenuation;
out vec3 vertexPositionScreenSpace;

void main() {
  sVaryingTexCoords = sTexCoords + vec2(0.5);
  
  float vertexSizeX = 30.0 * (1.0/800.0);
  float vertexSizeY = 30.0 * (1.0/600.0);
  
  gl_Position = sModelViewMatrix * sVertex;
  vertexPositionScreenSpace.z = gl_Position.z / gl_Position.w;
  gl_Position = sProjectionMatrix * gl_Position;
  vertexPositionScreenSpace.x = gl_Position.x / gl_Position.w;
  vertexPositionScreenSpace.y = gl_Position.y / gl_Position.w;
  
  gl_Position.z -= 0.01;
  gl_Position.x += (sTexCoords.s * vertexSizeX) * gl_Position.w;
  gl_Position.y += (sTexCoords.t * vertexSizeY) * gl_Position.w;
}