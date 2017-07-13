#version 120

uniform mat4 worldviewproj_matrix;
varying vec2 varyingTexCoord0;

void main()
{
//gl_Position = worldviewproj_matrix * vec4(gl_Vertex.xyz,1.0);
	gl_Position = ftransform();
	varyingTexCoord0 = gl_TexCoord[0].xy;  

}
