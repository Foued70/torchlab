in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

in vec3 vertexPositionScreenSpace;

uniform sampler2D textureMap_0; //dot texture
uniform sampler2D textureMap_1; //framebuffer depth map

float linearizeDepth(vec2 uv)
{
  float near = 0.0001; // camera z near
  float far = 1000.0; // camera z far
  float depthBuffer = texture(textureMap_1, uv).r;
  return (((2.0*near) / (far+near - depthBuffer*(far-near))) * (far-near)) + near;
}

void main () {
    vec2 p = sVaryingTexCoords * 2.0 - vec2(1.0);
    float r = sqrt(dot(p,p));
    
    vec2 screenPositionUV;
    screenPositionUV.x = (vertexPositionScreenSpace.x + 1.0) * 0.5;
    screenPositionUV.y = (vertexPositionScreenSpace.y + 1.0) * 0.5;
        
    if (dot(p,p) > r)
      discard;
    if (linearizeDepth(screenPositionUV) < 5.0)
      sFragColor = vec4(1.0, 0.0, 0.0, 1.0);
    else
      sFragColor = vec4(1.0);
}