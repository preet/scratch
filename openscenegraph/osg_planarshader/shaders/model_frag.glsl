//precision mediump float;

varying vec2 TexCoord0;

float calcIsLeft(vec2 p0,vec2 p1,vec2 p2)
{
    // 2d cross product
    float temp = (p1.x - p0.x)*(p2.y - p0.y) -
            (p2.x - p0.x)*(p1.y - p0.y);
    return temp;
}

void main()
{
    vec2 tc = TexCoord0;
//    gl_FragColor = vec4(1,1,1,1.0-TexCoord0.y);

//    float dx = tc.x;
//    float dy = tc.y;
//    float d = (dx*dx + dy*dy);
//    d = d*d;

    float d = 0.0;
    vec2 yintcept = vec2(0,0.9);
    vec2 xintcept = vec2(0.9,0);
    bool someState = (calcIsLeft(xintcept,yintcept,tc) > 0.0);

    if(someState)
    {   d = 1.0;   }

    if(tc.x < 0.05)
    {   d = 0.0;   }

    if(tc.y < 0.05)
    {   d = 0.0;   }

    gl_FragColor = vec4(d, d, d, 1);


}
