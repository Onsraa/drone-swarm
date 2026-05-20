// Instanced cube rendering for the ground-truth layer.
//
// Each instance is a unit-cube mesh whose center is `i_pos_scale.xyz`
// and whose side length is `i_pos_scale.w` (meters). The cube mesh
// itself carries vertex positions in [-0.5, +0.5] on each axis, so the
// world position is just (vertex * side) + center. Output color
// includes alpha so the Transparent3d phase blends the cube over
// whatever the local + global map layers drew underneath.

#import bevy_pbr::view_transformations::position_world_to_clip

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) i_pos_scale: vec4<f32>, // xyz = center, w = side length
    @location(4) i_color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
};

@vertex
fn vertex(v: Vertex) -> VertexOutput {
    let world_pos = v.position * v.i_pos_scale.w + v.i_pos_scale.xyz;
    var out: VertexOutput;
    out.clip_position = position_world_to_clip(world_pos);
    out.color = v.i_color;
    out.world_normal = v.normal;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    // Cheap diagonal lambert so cube faces have contrast and the
    // transparent volume reads as 3D, not flat.
    let n = normalize(in.world_normal);
    let light_dir = normalize(vec3<f32>(0.4, 1.0, 0.3));
    let lambert = max(dot(n, light_dir), 0.0);
    let shaded = in.color.rgb * (0.55 + 0.45 * lambert);
    return vec4<f32>(shaded, in.color.a);
}
