#version 120

// color varying for fragment shader
varying vec2 TexCoord1;

void main()
{
    TexCoord1 = gl_MultiTexCoord1.xy;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
