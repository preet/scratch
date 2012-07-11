#version 120

// generic vertex attributes
attribute vec4 a_position;
attribute vec4 a_normal;

// color varying for fragment shader
//varying mediump vec4 v_color;
varying vec4 v_color;

void main()
{
   gl_Position = gl_ModelViewProjectionMatrix * a_position;
   v_color = a_normal;
}
