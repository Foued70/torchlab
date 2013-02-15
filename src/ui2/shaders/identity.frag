void main() {
	sFragColor = sDefColor;
  sPickingData = uvec3((gl_PrimitiveID+1), objectID, meshID);
}