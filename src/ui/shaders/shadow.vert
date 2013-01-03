out vec3 sVaryingNormal;
out vec3 sVaryingLightDir;
out vec3 sEyeVector;
out float sAttenuation;

void main() {	
	sVaryingNormal = sNormalMatrix * sNormal;

	vec3 tempVertex = vec3(sModelViewMatrix * sVertex);
	sVaryingLightDir = vec3(sLightSource[0].position.xyz - tempVertex);
	sEyeVector = -tempVertex;
	
	float d = length(sVaryingLightDir);
	
	sAttenuation = 1.0 / ( sLightSource[0].constantAttenuation + 
	(sLightSource[0].linearAttenuation*d) + 
	(sLightSource[0].quadraticAttenuation*d*d) );
	

	gl_Position = sModelViewProjectionMatrix * sVertex;
}