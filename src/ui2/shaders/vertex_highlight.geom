layout (triangles) in;
layout (triangle_strip,  max_vertices=3 ) out;

in vec3 vertexCenterScreenSpace[3];

smooth out vec2 sVaryingTexCoords;
out vec3 vertexPositionScreenSpace;

void main() {

    float vertexSizeX = 20.0 * (1.0/float(screenWidth));
    float vertexSizeY = 20.0 * (1.0/float(screenHeight));

    for(int i = 0; i < gl_in.length(); i++)
    {
        vec4 pos = gl_in[i].gl_Position;

        // Bottom right corner
        gl_Position.x = pos.x + (1.0 * vertexSizeX) * pos.w;
        gl_Position.y = pos.y + (-1.0 * vertexSizeY) * pos.w;
        gl_Position.z = pos.z;
        gl_Position.w = pos.w;
        sVaryingTexCoords = vec2(1.0, 0.0);
        vertexPositionScreenSpace = vertexCenterScreenSpace[i];
        EmitVertex();

        // Top center
        gl_Position.x = pos.x;
        gl_Position.y = pos.y + (1.0 * vertexSizeY) * pos.w;
        gl_Position.z = pos.z;
        gl_Position.w = pos.w;
        sVaryingTexCoords = vec2(0.5, 1.0);
        vertexPositionScreenSpace = vertexCenterScreenSpace[i];
        EmitVertex();

        // Bottom left corner
        gl_Position.x = pos.x + (-1.0 * vertexSizeX) * pos.w;
        gl_Position.y = pos.y + (-1.0 * vertexSizeY) * pos.w;
        gl_Position.z = pos.z;
        gl_Position.w = pos.w;
        sVaryingTexCoords = vec2(0.0, 0.0);
        vertexPositionScreenSpace = vertexCenterScreenSpace[i];
        EmitVertex();

        

        
        
    }
    EndPrimitive();
}