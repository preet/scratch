#version 120
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

// attributes
attribute vec3 ModelPosition;

// uniforms
uniform mat4 BillboardRotate;

// varyings
varying mediump vec4 VertexColor;
varying vec2 TexCoord0;

void main()
{
//    gl_Position = gl_ProjectionMatrix * (gl_Vertex + vec4(gl_ModelViewMatrix[3].xyz,0));

    // rotate geometry about its model position center
    vec4 v_modelPosition = vec4(ModelPosition.xyz,1.0);
    vec4 v_xlateModelToWorld = gl_Vertex - v_modelPosition;
    vec4 v_alignedPosition = BillboardRotate * v_modelPosition;
    vec4 v_worldPosition = v_alignedPosition + v_xlateModelToWorld;

    // calculate final vertex color
    VertexColor = gl_Color;
    TexCoord0 = gl_MultiTexCoord0.xy;

    // calculate projected vertex position
    gl_Position = gl_ModelViewProjectionMatrix * v_worldPosition;
}
