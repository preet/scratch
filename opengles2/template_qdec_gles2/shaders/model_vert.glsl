// VERTEX SHADER

// notes:
// to maintain compatibility, the version
// preprocessor call needs to be added to the
// beginning of this file by the (cpu) compiler:
//
// "#version 100" for OpenGL ES 2 and
// "#version 120" (or higher) for desktop OpenGL

#ifdef GL_ES
    // vertex shader defaults for types are:
    // precision highp float;
    // precision highp int;
    // precision lowp sampler2D;
    // precision lowp samplerCube;
#else
    // with default (non ES) OpenGL shaders, precision
    // qualifiers aren't used -- we explicitly set them
    // to be defined as 'nothing' so they are ignored
    #define lowp
    #define mediump
    #define highp
#endif

// generic vertex attributes
attribute vec4 a_position;
attribute vec4 a_normal;

// model*view*projection transform
uniform mat4 xf_mvp;

// color varying for fragment shader
varying mediump vec4 v_color;

void main()
{
   gl_Position = xf_mvp * a_position;
   v_color = vec4(1,1,1,1);
}
