#version 120

uniform sampler2D texFront;
uniform sampler2D texRear;
varying vec2 texCoordFront;
varying vec2 texCoordRear;

void main()
{
	vec4 colFront = texture2D(texFront, texCoordFront);
	vec4 colRear = texture2D(texRear, texCoordRear);
	//colFront = vec4(0,0,0,0.5);
	//colRear = vec4(0,0,0,0.5);
	gl_FragColor = (vec4(colFront.xyz, 1)*1 + 1*vec4(colRear.xyz, 1));
}
