// GPU-instanced point-cloud billboard rendering.
//
// Every "instance" is a single point in world space; the vertex shader
// synthesises a camera-facing flat quad around it whose size is
// expressed in screen pixels (`i_pos_scale.w` = pixel radius). The
// fragment shader discards corners outside the unit disk so each dot
// reads as round, not square.
//
// The mesh attached to every layer entity is a 2-tri unit quad whose
// per-vertex `position.xy` carries the corner offset in [-1, +1].
// `position.z` and other attributes are ignored.

#import bevy_pbr::view_transformations::position_world_to_clip
#import bevy_pbr::mesh_view_bindings::view

struct Vertex {
    @location(0) position: vec3<f32>,    // quad corner offset in xy
    @location(1) normal: vec3<f32>,      // unused
    @location(2) uv: vec2<f32>,          // unused
    @location(3) i_pos_scale: vec4<f32>, // xyz = world pos, w = pixel radius
    @location(4) i_color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) quad_uv: vec2<f32>,
};

@vertex
fn vertex(v: Vertex) -> VertexOutput {
    let center_clip = position_world_to_clip(v.i_pos_scale.xyz);
    let corner = v.position.xy;
    let radius_px = max(v.i_pos_scale.w, 1.0);
    let viewport_size = view.viewport.zw;
    // NDC pixels: 2.0 / viewport. Pre-multiply by clip.w so the dot
    // stays constant on screen regardless of distance.
    let clip_offset = corner * radius_px * 2.0 / viewport_size;
    var out: VertexOutput;
    out.clip_position = vec4<f32>(
        center_clip.xy + clip_offset * center_clip.w,
        center_clip.z,
        center_clip.w,
    );
    out.color = v.i_color;
    out.quad_uv = corner;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    // Round dot: discard outside the unit disk.
    if (dot(in.quad_uv, in.quad_uv) > 1.0) {
        discard;
    }
    return in.color;
}
