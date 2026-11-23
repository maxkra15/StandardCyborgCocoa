//
//  DrawColorTexture.metal
//  Capture
//
//  Copyright © 2018 Standard Cyborg. All rights reserved.
//

#include <metal_stdlib>

using namespace metal;

struct Uniforms
{
    float3x3 transform;
    float alpha;
};

kernel void DrawColorTexture(texture2d<float, access::sample> colorTexture [[texture(0)]],
                             texture2d<float, access::write> resultTexture [[texture(1)]],
                             constant Uniforms *uniforms [[buffer(0)]],
                             uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= resultTexture.get_width() || gid.y >= resultTexture.get_height()) { return; }
    constexpr sampler colorSampler(coord::normalized, address::clamp_to_edge, filter::linear);
    
    float2 uv = (uniforms->transform * float3(float2(gid) + 0.5, 1.0)).xy;
    
    float4 color = colorTexture.sample(colorSampler, uv);
    color *= uniforms->alpha;
    
    resultTexture.write(color, gid);
}
