in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

in vec3 vertexPositionScreenSpace;

uniform sampler2D textureMap_0; //dot texture
uniform sampler2D textureMap_1; //framebuffer depth map

void main () {
    vec2 p = sVaryingTexCoords * 2.0 - vec2(1.0);
    float r = sqrt(dot(p,p));
    
    vec2 screenPositionUV;
    screenPositionUV.x = (vertexPositionScreenSpace.x + 1.0) * 0.5;
    screenPositionUV.y = (vertexPositionScreenSpace.y + 1.0) * 0.5;
    
    const float cullWiggleRoom = 0.002;
        
    if (dot(p,p) > r)
      discard;
    else if (texture(textureMap_1, screenPositionUV).r < vertexPositionScreenSpace.z && 
              abs(texture(textureMap_1, screenPositionUV).r - vertexPositionScreenSpace.z) > cullWiggleRoom)
      discard;
    
    else
      sFragColor = mix(vec4(0.0, 1.0, 1.0, 1.0), vec4(0.1, 0.0, 0.3, 1.0), pow(dot(p,p), 1.0/2.2));
      sPickingData = uvec3(0, 0, 0);
}
