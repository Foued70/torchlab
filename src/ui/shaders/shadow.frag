in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

void main () {
	sFragColor = ((sFrontMaterial.emission + sFrontMaterial.ambient * sLightModel.ambient)
			* sFrontMaterial.ambient) + (sLightSource[0].ambient * sFrontMaterial.ambient) * sAttenuation;
	
	vec3 N = normalize(sVaryingNormal);
	vec3 L = normalize(sVaryingLightDir);
	
	float lambertTerm = dot(N,L);
	
	if(lambertTerm > 0.0) {
		sFragColor += sLightSource[0].diffuse *
				sFrontMaterial.diffuse *
				lambertTerm * sAttenuation;
		
		vec3 E = normalize(sEyeVector);
		vec3 R = reflect(-L, N);
		
		float specular = pow( max(dot(R, E), 0.0),
				sFrontMaterial.shininess );
		
		sFragColor += sLightSource[0].specular *
		sFrontMaterial.specular * specular * sAttenuation;
	}

	sFragColor *= sDefColor;
}