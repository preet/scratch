precision mediump float;

// varying to define color
varying vec4 v_texcoord;

uniform sampler2D s_tex;

void main()
{
    gl_FragColor = texture2D(s_tex,v_texcoord);
}
