#version 150

in vec4 sVertex;
in vec3 sNormal;
in vec2 sTexCoords;

void main() {
  // gl_Position = sModelViewProjectionMatrix * sVertex;
	gl_Position = sVertex;
}