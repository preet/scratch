//precision mediump float;

// varying to define color
varying vec2 TexCoord1;

void main()
{
//    float cVal = fract(TexCoord1.y*80.0); //sin(TexCoord1.y*16.0);
//    if(cVal > 0.5)
//    {   cVal = 1.0;   }
//    else
//    {   cVal = 0.0;   }

//    gl_FragColor = vec4(cVal,cVal,cVal,1.0);


    gl_FragColor = vec4(TexCoord1.y,1.0,1.0-TexCoord1.y,1.0);
}
