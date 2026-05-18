// True GPU-instanced voxel rendering. The CPU side uploads one
// `InstanceData` per occupied cell into a vertex buffer with
// `step_mode = Instance`. The vertex shader reads per-instance position +
// scale + color from that buffer and offsets a shared unit-cube mesh.

#import bevy_pbr::view_transformations::position_world_to_clip

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) i_pos_scale: vec4<f32>,
    @location(4) i_color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
};

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    let world_pos = vertex.position * vertex.i_pos_scale.w + vertex.i_pos_scale.xyz;
    var out: VertexOutput;
    out.clip_position = position_world_to_clip(world_pos);
    out.color = vertex.i_color;
    out.world_normal = vertex.normal;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(0.3, 1.0, 0.2));
    let n_dot_l = max(dot(normalize(in.world_normal), light_dir), 0.0);
    let ambient = 0.4;
    let intensity = ambient + n_dot_l * 0.6;
    return vec4<f32>(in.color.rgb * intensity, in.color.a);
}
