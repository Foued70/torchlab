in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

uniform usampler2D textureUnit0; // Picking pass as rendered to texture

void main () {
  float lineThickness = 2.0; //Line thickness in pixels
  vec2 offset = vec2(lineThickness/(float(screenWidth)*2.0), lineThickness/(float(screenHeight)*2.0));

  if      ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st + vec2(offset.s, 0.0)) )
    sFragColor = vec4(1.0);
  else if ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st - vec2(offset.s, 0.0)) )
    sFragColor = vec4(1.0);
  else if ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st + vec2(0.0, offset.t)) )
    sFragColor = vec4(1.0);
  else if ( texture(textureUnit0, sVaryingTexCoords.st) != 
            texture(textureUnit0, sVaryingTexCoords.st - vec2(0.0, offset.t)) )
    sFragColor = vec4(1.0);
  else 
      discard;
}