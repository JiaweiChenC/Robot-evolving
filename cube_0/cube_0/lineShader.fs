#version 330 core

// Interpolated values from the vertex shaders
// in vec3 fragmentColor;
// don't need color for nwo
// Ouput data
out vec4 color;

void main(){

    // Output color = color specified in the vertex shader,
    // interpolated between all 3 surrounding vertices
    // color = fragmentColor;
    color = vec4(0.01f, 0.01f, 0.01f, 1.0f);

}
