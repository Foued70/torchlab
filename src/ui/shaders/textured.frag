in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

vec3 colorTrianglesByID(in int _id) {
  vec3 color = vec3(0.0, 0.0, 0.0);
  
  if (_id == 0)
    color = vec3(1.0, 0.0, 0.0);
  else if (_id == 1)
    color = vec3(1.0, 1.0, 0.0);
  else if (_id == 2)
    color = vec3(0.0, 1.0, 0.0);
  else if (_id == 3)
    color = vec3(0.0, 1.0, 1.0);
  else if (_id == 4)
    color = vec3(0.0, 0.0, 1.0);
  else
    color = vec3(1.0, 1.0, 1.0);
  
  return color;
}

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
  
  
  sFragColor *= texture(textureUnit, sVaryingTexCoords.st);
  
  sFragColor.rgb *= colorTrianglesByID(int(gl_PrimitiveID));
  sPickingData = uvec3(objectID, meshID, gl_PrimitiveID);
}




