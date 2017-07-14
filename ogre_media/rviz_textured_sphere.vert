#version 120

//uniform sampler2D texFront;
//uniform sampler2D texRear;
//uniform vec4 gl_MultiTexCoord0;
//uniform vec4 gl_MultiTexCoord1;

varying vec2 texCoordFront;
varying vec2 texCoordRear;

void main()
{
	gl_Position = ftransform();
	texCoordFront = gl_MultiTexCoord0.st;  
	texCoordRear = gl_MultiTexCoord1.st;
}
