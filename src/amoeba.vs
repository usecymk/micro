#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;

// Input uniform values
uniform mat4 mvp;
uniform mat4 matModel;

// Output to fragment shader
out vec3 fragPosition;
out vec3 fragNormal;
out vec4 fragColor;

void main()
{
    fragPosition = vec3(matModel * vec4(vertexPosition, 1.0));
    
    // Using the upper 3x3 of the model matrix is usually fine, 
    // but ensure your C++ code passes a non-scaled model matrix if possible.
    fragNormal = normalize(mat3(matModel) * vertexNormal); 
    
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}