#version 120

uniform sampler2D texFront;
uniform sampler2D texRear;
//uniform mat4 worldviewproj_matrix;
varying vec2 texCoordFront;
varying vec2 texCoordRear;

void main()
{
	gl_Position = ftransform();
	texCoordFront = gl_MultiTexCoord0.st*vec2(2,1)+vec2(0.0,0.0);  
	texCoordRear = gl_MultiTexCoord1.st*vec2(2,1)-vec2(1.0,0.0);
}
