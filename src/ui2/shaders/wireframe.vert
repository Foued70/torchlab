out vec3 sVaryingNormal;
out vec3 sVaryingLightDir;
out vec3 sEyeVector;
out float sAttenuation;

void main() {
  gl_Position = vec4(sVertex.x, sVertex.z, -1.0, 1.0);
  sVaryingTexCoords = sTexCoords;
  sVaryingTexCoords.x = sVaryingTexCoords.x;
  sVaryingTexCoords.y = 1.0 - sVaryingTexCoords.y;
}
