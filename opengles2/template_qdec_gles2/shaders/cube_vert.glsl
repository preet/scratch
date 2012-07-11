#version 120

// generic vertex attributes
attribute vec4 a_position;
attribute vec4 a_color;

// model*view*projection transform
uniform mat4 xf_mvp;

// color varying for fragment shader
varying mediump vec4 v_color;

void main()
{
   gl_Position = xf_mvp * a_position;
   v_color = a_color;
}
