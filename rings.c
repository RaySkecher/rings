#define NUM_RINGS 5

//ring
typedef struct {
    Vec3 center;
    Vec3 normal;
    fp_t inner_radius;
    fp_t outer_radius;
    fp_t ellipse_ratio;
    Material material;
} Ring;

Ring g_rings[NUM_RINGS];

void setup_rings() {
    Vec3 jupiter_center = (Vec3){F(2), F(0), F(-1)};  // Position of Jupiter

    // Manually customized ring sizes and colors
   fp_t inner_radii[NUM_RINGS]     = {F(0.4), F(0.65)};
    fp_t outer_radii[NUM_RINGS]     = {F(0.6), F(0.85)};
    fp_t ellipse_ratios[NUM_RINGS]  = {F(0.7), F(0.7)};  // Elliptical shape
    Vec3 colors[NUM_RINGS] = {
        {F(0.9), F(0.5), F(0.1)},   // Inner: rusty orange
        {F(0.8), F(0.8), F(0.5)}    // Outer: pale yellow
    };
    for (int i = 0; i < NUM_RINGS; ++i) {
         g_rings[i].center = jupiter_center;

        // All rings lie in roughly the same flat orbital plane
        g_rings[i].normal = vec_norm((Vec3){F(0.05), F(1), F(0.02)});

        g_rings[i].inner_radius = inner_radii[i];
        g_rings[i].outer_radius = outer_radii[i];
        g_rings[i].ellipse_ratio = ellipse_ratios[i];

        g_rings[i].material = (Material){
            .color = colors[i],
            .is_light = 0
        };
    }
}

//inside Intersection intersect_scene(Ray r):
// Ring intersection
    for (size_t i = 0; i < NUM_RINGS; ++i) {
        int32_t t = intersect_ring(r, g_rings[i]);
        if (t < result.t) {
            result.t = t;
            result.hit_type = 2; // 2 = ring
            result.hit_index = (int)i;
        }
    }

int32_t intersect_ring(Ray r, Ring ring) {
    int32_t denom = vec_dot(ring.normal, r.dir);
    if (denom > -FP_EPS && denom < FP_EPS) return FP_INF;

    int32_t t = div_fp(vec_dot(ring.normal, vec_sub(ring.center, r.orig)), denom);
    if (t <= FP_EPS) return FP_INF;

    Vec3 hit_point = vec_add(r.orig, vec_scale(r.dir, t));
    Vec3 d = vec_sub(hit_point, ring.center);
    // Elliptical ring check (assume ellipse lies in XZ-plane)
    int32_t x = d.x;
    int32_t z = d.z;

    int32_t a = ring.outer_radius;
    int32_t b = mul(a, ring.ellipse_ratio);  // semi-minor axis

    int32_t x_term = div_fp(mul(x, x), mul(a, a));
    int32_t z_term = div_fp(mul(z, z), mul(b, b));
    int32_t ellipse_outer = x_term + z_term;

    int32_t a_inner = ring.inner_radius;
    int32_t b_inner = mul(a_inner, ring.ellipse_ratio);

    int32_t x_term_inner = div_fp(mul(x, x), mul(a_inner, a_inner));
    int32_t z_term_inner = div_fp(mul(z, z), mul(b_inner, b_inner));
    int32_t ellipse_inner = x_term_inner + z_term_inner;

    if (ellipse_outer > ONE || ellipse_inner < ONE) return FP_INF;

    return t;
}

//call setup_rings in main():
    setup_rings();
