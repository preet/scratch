#version 120

// generic vertex attributes
attribute vec4 a_position;
attribute vec2 a_texcoord;

// model*view*projection transform
uniform mat4 xf_mvp;

// color varying for fragment shader
varying mediump vec2 v_texcoord;

void main()
{
   gl_Position = xf_mvp * a_position;
   v_texcoord = a_texcoord;
}
