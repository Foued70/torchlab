#version 330

// Hello World shader, does nothing woo!
in  vec3 Position;

void main(void) {    
	gl_Position = vec4(Position, 1.0);
}