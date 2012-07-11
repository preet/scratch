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

// varyings
varying mediump vec4 VertexColor;

// uniforms
uniform vec4 Color;

void main()
{
    // default params
    vec4 LightPosition = vec4(0.0, 0.0, 0.0, 1.0);
    vec3 LightColor = vec3(1.0, 1.0, 1.0);
    vec3 DiffuseColor = vec3(Color);

    // find the vector from the given vertex to the light source
    vec4 vertexInWorldSpace = gl_ModelViewMatrix * vec4(gl_Vertex);
    vec3 normalInWorldSpace = normalize(gl_NormalMatrix * gl_Normal);
    vec3 lightDirn = normalize(vec3(LightPosition-vertexInWorldSpace));

    // calculate final vertex color
    VertexColor = vec4(DiffuseColor * max(dot(lightDirn,normalInWorldSpace),0.0), Color.w);

    // calculate projected vertex position
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

}
