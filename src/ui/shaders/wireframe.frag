in vec3 sVaryingNormal;
in vec3 sVaryingLightDir;
in vec3 sEyeVector;
in float sAttenuation;

uniform sampler2D textureMap_0;
uniform usampler2D textureMap_1;

void main () {
    vec3 wireframe = vec3( 0.0 );
    vec2 lineThickness = vec2(1.0/400.0, 1.0/600.0);
   
    if (  texture(textureMap_1, sVaryingTexCoords.st) != 
          texture(textureMap_1, sVaryingTexCoords.st + lineThickness) )
      wireframe += vec3(1.0);
    
    vec2 photoUVs = vec2(sVaryingTexCoords.t, sVaryingTexCoords.s);
    
    sFragColor.rgb = texture(textureMap_0, photoUVs).rgb + wireframe;
    sFragColor.a = 0.5;
    sPickingData = uvec3(objectID, meshID, gl_PrimitiveID);
}


  





