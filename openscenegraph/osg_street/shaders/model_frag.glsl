//precision mediump float;

// varying to define color
varying vec2 TexCoord1;

void main()
{
	gl_FragColor = vec4(TexCoord1.y,1.0,1.0-TexCoord1.y,1.0);
}
