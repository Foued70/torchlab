in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

uniform usampler2D textureUnit0; // Picking pass as rendered to texture

void main () {
  float lineThickness = 2.0; //Line thickness in pixels
  vec2 offset = vec2(lineThickness/(float(screenWidth)*2.0), lineThickness/(float(screenHeight)*2.0));
  vec4 wireframeColor = vec4(0.76, 0.88, 0.86, 1.0);

  if      ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st + vec2(offset.s, 0.0)) )
    sFragColor = wireframeColor;
  else if ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st - vec2(offset.s, 0.0)) )
    sFragColor = wireframeColor;
  else if ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st + vec2(0.0, offset.t)) )
    sFragColor = wireframeColor;
  else if ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st - vec2(0.0, offset.t)) )
    sFragColor = wireframeColor;
  else 
      discard;
}