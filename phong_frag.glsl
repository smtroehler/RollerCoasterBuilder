
uniform vec3 ka;
uniform vec3 kd;
uniform vec3 ks;
uniform float s;

uniform vec3 light;
uniform float inten;

varying vec3 normal;
varying vec3 vert;

void main()
{
   vec3 n = normalize(normal);
   vec3 finalColor = vec3(0.0,0.0,0.0);


   vec3 l = normalize(light - vert); 
   vec3 color_d  = kd * max(dot(l, n), 0);
   vec3 e = normalize(-vert);
   vec3 h = normalize(l + e); 
   vec3 color_s = ks * pow(max(dot(h, n), 0), s);
   vec3 color = ka + color_d + color_s;
   finalColor = inten * color;
   
   
   
   
   gl_FragColor = vec4(finalColor.r, finalColor.g, finalColor.b, 1.0);

}
