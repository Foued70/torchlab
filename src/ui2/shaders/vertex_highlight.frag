out vec4 sFragColor;
out uvec3 sPickingData;
smooth in vec2 sVaryingTexCoords;

in vec3 vertexPositionScreenSpace;

uniform sampler2D textureUnit0; //framebuffer depth map

void main () {
    vec2 p = (sVaryingTexCoords * vec2(2.0)) - vec2(1.0);
    p /= vec2(0.3);
    float r = sqrt(dot(p,p));
    
    vec2 screenPositionUV;
    screenPositionUV.x = (vertexPositionScreenSpace.x + 1.0) * 0.5;
    screenPositionUV.y = (vertexPositionScreenSpace.y + 1.0) * 0.5;
    
    const float cullWiggleRoom = 0.002;
    float worldDepth = texture(textureUnit0, screenPositionUV).r;
      
    if (dot(p,p) > r)
      discard;
    else if (worldDepth < vertexPositionScreenSpace.z && 
           abs(worldDepth - vertexPositionScreenSpace.z) > cullWiggleRoom)
      discard;
    else
      sFragColor = mix(vec4(0.5, 0.9, 0.9, 1.0), vec4(0.1, 0.2, 0.5, 1.0), clamp(r, 0.0, 1.0));
}
