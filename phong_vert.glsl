attribute vec4 vertPos; // in object space
attribute vec3 vertNor; // in object space
uniform mat4 P;
uniform mat4 MV;

varying vec3 normal;
varying vec3 vert;
void main()
{
   gl_Position = P * MV * vertPos;
   normal = (MV * vec4(vertNor,0)).xyz;
   vert = (MV * vertPos).xyz;
}
