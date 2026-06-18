#version 330

in vec3 fragPosition;
in vec3 fragNormal;
in vec4 fragColor;

uniform vec3 viewPos;
uniform float time;

out vec4 finalColor;

void main()
{
    vec3 viewDir = normalize(viewPos - fragPosition);
    
    // 1. Soften the Rim: Use pow() instead of smoothstep for a more organic glow
    float fresnel = pow(1.0 - max(dot(viewDir, fragNormal), 0.0), 3.0);
    
    // 2. Soft Lighting: Prevents "faceted" look by basing brightness on normal angle
    float lightIntensity = 0.5 + 0.5 * max(dot(fragNormal, vec3(0.0, 1.0, 0.0)), 0.0);
    
    // 3. Shimmer
    float shimmer = (sin(time * 4.0 + fragPosition.y * 6.0) * 0.5 + 0.5) * 0.15;
    
    // 4. Mixing: Use the fresnel to drive the edge transition, 
    // and multiply by lightIntensity to give depth to the surface
    vec4 baseCol = vec4(fragColor.rgb * lightIntensity, fragColor.a);
    vec4 edgeCol = vec4(fragColor.rgb + shimmer, min(fragColor.a * 2.0, 1.0));
    
    finalColor = mix(baseCol, edgeCol, fresnel * 0.7); // 0.7 limits the max brightness of the rim
}