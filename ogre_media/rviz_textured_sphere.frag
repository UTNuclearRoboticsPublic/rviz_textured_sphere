#version 120

uniform sampler2D Texture0;
uniform sampler2D Texture1;
varying vec2 texCoordFront;
varying vec2 texCoordRear;


void main()
{
	vec4 colFront = texture2D(Texture0, texCoordFront);
	vec4 colRear = texture2D(Texture1, texCoordRear);
	gl_FragColor = colFront+colRear;
//	gl_FragColor = colFront;
//(colFront + colRear)/2.0;
	
}
