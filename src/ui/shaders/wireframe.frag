in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

uniform sampler2D textureMap_0;
uniform usampler2D textureMap_1;

void main () {
    vec3 wireframe = vec3( 0.0 );
    float lineThickness = 2.0; //Line thickness in pixels
    vec2 offset = vec2(lineThickness/(400.0*2.0), lineThickness/(600.0*2.0));

    if (      texture(textureMap_1, sVaryingTexCoords.st) != 
              texture(textureMap_1, sVaryingTexCoords.st + vec2(offset.s, 0.0)) )
      wireframe = vec3(1.0);
    else if ( texture(textureMap_1, sVaryingTexCoords.st) != 
              texture(textureMap_1, sVaryingTexCoords.st - vec2(offset.s, 0.0)) )
      wireframe = vec3(1.0);
    else if ( texture(textureMap_1, sVaryingTexCoords.st) != 
              texture(textureMap_1, sVaryingTexCoords.st + vec2(0.0, offset.t)) )
      wireframe = vec3(1.0);
    else if ( texture(textureMap_1, sVaryingTexCoords.st) != 
              texture(textureMap_1, sVaryingTexCoords.st - vec2(0.0, offset.t)) )
      wireframe = vec3(1.0);
    
    vec2 photoUVs = vec2(sVaryingTexCoords.t, sVaryingTexCoords.s);
    
    sFragColor.rgb = texture(textureMap_0, photoUVs).rgb + wireframe;
    sFragColor.a = 0.5;
    sPickingData = uvec3(objectID, meshID, gl_PrimitiveID);
    sPickingData = uvec3(0, 0, 0);
}


  





