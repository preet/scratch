#version 120

varying vec4 VertexColor;

void main()
{
    VertexColor = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
