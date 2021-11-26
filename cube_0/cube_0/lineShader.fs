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
    color = vec4(0.4f, 0.9f, 0.4f, 1.0f);

}
