#version 120

// color varying for fragment shader
varying vec4 v_color;

void main()
{
   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
