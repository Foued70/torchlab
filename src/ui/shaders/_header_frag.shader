out vec4 sFragColor;
out uvec3 triangleID;

smooth in vec2 sVaryingTexCoords;
uniform sampler2D textureUnit;
uniform sampler2D normalMap;
uniform sampler2D specularMap;