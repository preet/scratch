#version 120

varying vec2 TexCoord0;
uniform sampler2D Texture;

float rand(vec2 co){
    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

void main()
{
    vec4 tileAlpha = texture2D(Texture,TexCoord0);
    vec4 tileOpacity = vec4(1,1,1,1)-tileAlpha;
    vec4 landColor = vec4(0.7,0.9,0.5,1.0);
    vec4 waterColor = vec4(0.2,0.4,0.8,1.0);
    vec4 texColor = landColor*tileAlpha.aaaa + waterColor*tileOpacity.aaaa;
    gl_FragColor = texColor;
//    gl_FragColor = vec4(TexCoord0.x,TexCoord0.y,0,1.0);

//    float ns = rand(TexCoord0);
//    gl_FragColor = vec4(ns,ns,0.5,1.0);

//    vec2 position = ( gl_FragCoord.xy / 100.0 ) + 1.0;

//    float rand = mod(fract(sin(dot(position, vec2(12.9898,78.233))) * 43758.5453), 1.0);

//    gl_FragColor = vec4( rand, rand, rand, 1.0);
}
