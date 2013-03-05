out vec4 sFragColor;
out uvec3 sPickingData;
smooth in vec2 sVaryingTexCoords;

in vec3 vertexPositionScreenSpace;

uniform sampler2D textureUnit0; //framebuffer depth map

void main () {
    vec2 p = sVaryingTexCoords * 2.0 - vec2(1.0);
    p /= vec2(0.3);
    float r = sqrt(dot(p,p));
    
    vec2 screenPositionUV;
    screenPositionUV.x = (vertexPositionScreenSpace.x + 1.0) * 0.5;
    screenPositionUV.y = (vertexPositionScreenSpace.y + 1.0) * 0.5;
    
    const float cullWiggleRoom = 0.002;
        
    if (dot(p,p) > r)
      discard;
    else if (texture(textureUnit0, screenPositionUV).r < vertexPositionScreenSpace.z && 
              abs(texture(textureUnit0, screenPositionUV).r - vertexPositionScreenSpace.z) > cullWiggleRoom)
      discard;
    
    else
      sFragColor = mix(vec4(0.0, 1.0, 1.0, 1.0), vec4(0.1, 0.0, 0.3, 1.0), r);

}
