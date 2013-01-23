void main() {
	sFragColor = sDefColor;
	//triangleID = vec4(0.0, sFaceIndex, 0.0, 1.0);
  triangleID = uvec3((gl_PrimitiveID+1), 0, 0);
}