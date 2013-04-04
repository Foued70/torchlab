#version 150

struct sMaterialParams {
 vec4 emission;
 vec4 ambient;
 vec4 diffuse;
 vec4 specular;
 float shininess;
};
		
struct sLightParams {
 vec4 ambient;
 vec4 diffuse;
 vec4 specular;
 vec4 position;
 float constantAttenuation;
 float linearAttenuation;
 float quadraticAttenuation;
};
		
struct sLightModelParameters {
 vec4 ambient;
};

uniform uint objectID;
uniform uint submeshStart;

uniform uint screenWidth;
uniform uint screenHeight;

uniform vec4 sDefColor;
uniform mat4 sModelViewMatrix;
uniform mat4 sProjectionMatrix;
uniform mat4 sModelViewProjectionMatrix;
uniform mat3 sNormalMatrix;
uniform sMaterialParams sFrontMaterial;
uniform sLightParams sLightSource[7];
uniform sLightModelParameters sLightModel;
