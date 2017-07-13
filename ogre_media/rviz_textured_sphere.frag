#version 120

uniform sampler2D tex0;
varying vec2 varyingTexCoord0;

void main()
{
	gl_FragColor = texture2D(tex0, varyingTexCoord0);
}
