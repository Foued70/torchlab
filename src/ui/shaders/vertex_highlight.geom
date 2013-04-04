layout (triangles) in;
layout (triangle_strip,  max_vertices=3 ) out;

uniform uint selectedVertexCount;
uniform vec3 selectedVertexPosition;

in vec3 vertexCenterScreenSpace[3];
in float vertexSelected[3]; // Can't pass bools around. 0.0 = false, 1.0 = true

smooth out vec2 sVaryingTexCoords;
out vec3 vertexPositionScreenSpace;
out float highlightVertex; // Can't pass bools around. 0.0 = false, 1.0 = true

void main() {

    vec2 vertexSize;
    vertexSize.x = 20.0 * (1.0/float(screenWidth));
    vertexSize.y = 20.0 * (1.0/float(screenHeight));

    
    for(int i = 0; i < gl_in.length(); i++)
    {
        vec4 pos = gl_in[i].gl_Position;

        // Making gl_PrimitiveID a vertex draw counter
        gl_PrimitiveID = (gl_PrimitiveIDIn * 3) + i;

        vec2 animatedVertexSize = vertexSize;
        if (vertexSelected[i] > 0.5) {
          animatedVertexSize *= vec2(3.0);
        }

        // Bottom right corner
        gl_Position.x = pos.x + (1.0 * animatedVertexSize.x) * pos.w;
        gl_Position.y = pos.y + (-1.0 * animatedVertexSize.y) * pos.w;
        gl_Position.z = pos.z;
        gl_Position.w = pos.w;
        sVaryingTexCoords = vec2(1.0, 0.0);
        vertexPositionScreenSpace = vertexCenterScreenSpace[i];
        highlightVertex = vertexSelected[i];
        EmitVertex();

        // Top center
        gl_Position.x = pos.x;
        gl_Position.y = pos.y + (1.0 * animatedVertexSize.y) * pos.w;
        gl_Position.z = pos.z;
        gl_Position.w = pos.w;
        sVaryingTexCoords = vec2(0.5, 1.0);
        vertexPositionScreenSpace = vertexCenterScreenSpace[i];
        highlightVertex = vertexSelected[i];
        EmitVertex();

        // Bottom left corner
        gl_Position.x = pos.x + (-1.0 * animatedVertexSize.x) * pos.w;
        gl_Position.y = pos.y + (-1.0 * animatedVertexSize.y) * pos.w;
        gl_Position.z = pos.z;
        gl_Position.w = pos.w;
        sVaryingTexCoords = vec2(0.0, 0.0);
        vertexPositionScreenSpace = vertexCenterScreenSpace[i];
        highlightVertex = vertexSelected[i];
        EmitVertex();
    }
    EndPrimitive();
}