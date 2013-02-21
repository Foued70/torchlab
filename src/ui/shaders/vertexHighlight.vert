out vec3 sVaryingNormal;
out vec3 sVaryingLightDir;
out vec3 sEyeVector;
out float sAttenuation;
out vec3 vertexPositionScreenSpace;

void main() {
  sVaryingTexCoords = sTexCoords + vec2(0.5);
  
  float vertexSizeX = 13.0 * (1.0/400.0);
  float vertexSizeY = 13.0 * (1.0/600.0);
  
  gl_Position = sModelViewProjectionMatrix * sVertex;
  vertexPositionScreenSpace.xy = gl_Position.xy / gl_Position.w;
  vertexPositionScreenSpace.z = ((gl_Position.z/gl_Position.w)+1.0) / 2.0;

  gl_Position.x += (sTexCoords.s * vertexSizeX) * gl_Position.w;
  gl_Position.y += (sTexCoords.t * vertexSizeY) * gl_Position.w;
}
