attribute vec4 vPosition;
uniform mat4 xfModelViewProj;

void main()
{
   gl_Position = xfModelViewProj * vPosition;
}
