//
//  shader.hpp
//  LearnOpengl
//
//  Created by Jiawei Chen on 11/9/21.
//

#version 330 core
out vec4 FragColor;

in vec2 TexCoord;

// texture samplers
uniform sampler2D texture1;
uniform sampler2D texture2;
void main()
{
    FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord), 0.5);
    //FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
}
