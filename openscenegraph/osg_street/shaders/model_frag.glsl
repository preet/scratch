//precision mediump float;

// varying to define color
varying vec2 TexCoord1;

void main()
{
    float multiplier = 1.0; //80.0;
    float cVal = fract(TexCoord1.y*multiplier); //sin(TexCoord1.y*16.0);
//    if(cVal > 0.5)
//    {   cVal = 1.0;   }
//    else
//    {   cVal = 0.0;   }

    gl_FragColor = vec4(1.0,cVal,cVal,1.0);
}
