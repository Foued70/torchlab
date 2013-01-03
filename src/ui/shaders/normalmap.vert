out vec3 sVaryingLightDir;
out vec3 sEyeVector;

void main() {
	sVaryingTexCoords = sTexCoords;
	
	vec3 tangent;
	vec3 c1 = cross(sNormal, vec3(0.0, 0.0, 1.0)); 
	vec3 c2 = cross(sNormal, vec3(0.0, 1.0, 0.0)); 
	
	if (length(c1) > length(c2)) {
		tangent = normalize(c1);	
	} else {
		tangent = normalize(c2);
	}
	
	vec3 n = normalize(sNormalMatrix * sNormal);
	vec3 t = normalize(sNormalMatrix * tangent);
	vec3 b = cross(n, t);
	
	vec3 tempVertex = vec3(sModelViewMatrix * sVertex);
	vec3 tmpVec = sLightSource[0].position.xyz - tempVertex;
	
	sVaryingLightDir.x = dot(tmpVec, t);
	sVaryingLightDir.y = dot(tmpVec, b);
	sVaryingLightDir.z = dot(tmpVec, n);
	
	tmpVec = -tempVertex;
	sEyeVector.x = dot(tmpVec, t);
	sEyeVector.y = dot(tmpVec, b);
	sEyeVector.z = dot(tmpVec, n);
	
	gl_Position = sModelViewProjectionMatrix * sVertex;
}