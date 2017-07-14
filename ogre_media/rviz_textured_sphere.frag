#version 120

uniform sampler2D texFront;
uniform sampler2D texRear;
varying vec2 texCoordFront;
varying vec2 texCoordRear;


void main()
{
	vec4 colFront = texture2D(texFront, texCoordFront);
	gl_FragColor = colFront;


	vec4 colRear = texture2D(texRear, texCoordRear);
//	gl_FragColor = colFront+colRear;
	gl_FragColor = colFront;
//(colFront + colRear)/2.0;
	
}
