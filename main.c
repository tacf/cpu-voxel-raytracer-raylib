#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"

// -----------------------------------------------------------------------------
// Tutorial overview
// -----------------------------------------------------------------------------
// This sample renders a small voxel world using CPU ray traversal
// (Amanatides-Woo 3D DDA):
// 1) Build a tiny voxel scene in RAM.
// 2) Build one camera ray per output pixel.
// 3) Intersect each ray against the grid AABB.
// 4) Traverse voxel-to-voxel with DDA until hit/exit.
// 5) Write color into a CPU RGBA buffer.
// 6) Upload that CPU buffer into a raylib texture.
// 7) Draw texture fullscreen and draw a runtime diagnostics overlay.

enum {
    // CPU ray buffer resolution.
    IMG_W = 320,
    IMG_H = 180,

    // Voxel world dimensions.
    GRID_X = 24,
    GRID_Y = 16,
    GRID_Z = 24,
    GRID_SIZE = GRID_X * GRID_Y * GRID_Z,
};

// Scale for overlay text and controls.
static const float UI_FONT_SCALE = 1.2f;

typedef struct {
    int x;
    int y;
    int z;
} IVec3;

// Per-frame traversal diagnostics shown in the overlay.
typedef struct {
    int rays;
    int rays_entered_grid;
    int hits;
    int total_steps;
    int max_steps;
    float avg_steps_per_ray;
    float hit_ratio;
    float rays_per_sec;
    float steps_per_sec;
} FrameStats;

// Result returned by one ray traversal.
typedef struct {
    bool hit;
    bool entered_grid;
    int steps;
    Vector3 col;
} TraceResult;

// Global app state:
// - `pixels`: CPU-side RGBA render target (one color per ray/pixel).
// - `voxels`: tiny tutorial voxel scene (0 = empty, non-zero = material id).
// - runtime fields for timing, camera mode, and diagnostics overlay.
typedef struct {
    Texture2D ray_texture;
    Color pixels[IMG_W * IMG_H];
    uint8_t voxels[GRID_SIZE];

    float time_s;
    bool freeze_camera;
    bool request_quit;

    FrameStats frame_stats;
    float frame_ms;
    float fps_smooth;
} AppState;

// Pre-normalized directional light used for simple lambert shading.
static const Vector3 LIGHT_DIR = { 0.46608496f, 0.8474272f, 0.25422817f };
static AppState g_state;

static inline int clamp_i32(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float clamp_f32(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Convert 3D voxel coords to linear index.
// Can be optimized by using z-order curve algorithm
static inline int voxel_index(int x, int y, int z) {
    return x + y * GRID_X + z * GRID_X * GRID_Y;
}

static inline bool inside_grid(int x, int y, int z) {
    return x >= 0 && x < GRID_X && y >= 0 && y < GRID_Y && z >= 0 && z < GRID_Z;
}

// Write one voxel if coordinates are valid.
static inline void set_voxel(int x, int y, int z, uint8_t value) {
    if (inside_grid(x, y, z)) {
        g_state.voxels[voxel_index(x, y, z)] = value;
    }
}

// Build tutorial scene:
// - ground plane
// - red column
// - green wall
// - blue column
static void build_scene(void) {
    memset(g_state.voxels, 0, sizeof(g_state.voxels));

    for (int z = 0; z < GRID_Z; z++) {
        for (int x = 0; x < GRID_X; x++) {
            set_voxel(x, 0, z, 1);
        }
    }

    for (int y = 1; y <= 5; y++) {
        set_voxel(8, y, 8, 2);
        set_voxel(9, y, 8, 2);
        set_voxel(8, y, 9, 2);
        set_voxel(9, y, 9, 2);
    }

    for (int y = 1; y <= 3; y++) {
        for (int x = 14; x <= 18; x++) {
            set_voxel(x, y, 14, 3);
        }
    }

    for (int y = 1; y <= 7; y++) {
        set_voxel(17, y, 6, 4);
    }
}

static Vector3 sample_voxel_color(uint8_t id) {
    switch (id) {
        case 1: return (Vector3){ 0.28f, 0.30f, 0.33f };
        case 2: return (Vector3){ 0.95f, 0.30f, 0.18f };
        case 3: return (Vector3){ 0.15f, 0.75f, 0.35f };
        case 4: return (Vector3){ 0.20f, 0.45f, 0.95f };
        default: return (Vector3){ 1.0f, 1.0f, 1.0f };
    }
}

// Clip [tmin, tmax] interval against one axis-aligned slab.
// For nearly parallel rays, hit requires origin to be already inside slab.
static bool axis_slab(float orig, float dir, float mn, float mx, float* tmin, float* tmax) {
    if (fabsf(dir) < 1e-6f) {
        return !(orig < mn || orig > mx);
    }

    const float inv = 1.0f / dir;
    float t_a = (mn - orig) * inv;
    float t_b = (mx - orig) * inv;
    if (t_a > t_b) {
        const float tmp = t_a;
        t_a = t_b;
        t_b = tmp;
    }

    *tmin = fmaxf(*tmin, t_a);
    *tmax = fminf(*tmax, t_b);
    return true;
}

// Ray vs grid AABB intersection.
// Returns entry/exit parametric distance along the ray.
static bool ray_aabb(Vector3 ro, Vector3 rd, float* out_t0, float* out_t1) {
    float tmin = -1e30f;
    float tmax = 1e30f;

    if (!axis_slab(ro.x, rd.x, 0.0f, (float) GRID_X, &tmin, &tmax)) return false;
    if (!axis_slab(ro.y, rd.y, 0.0f, (float) GRID_Y, &tmin, &tmax)) return false;
    if (!axis_slab(ro.z, rd.z, 0.0f, (float) GRID_Z, &tmin, &tmax)) return false;
    if (tmax < fmaxf(tmin, 0.0f)) return false;

    *out_t0 = tmin;
    *out_t1 = tmax;
    return true;
}

// Core algorithm: Amanatides-Woo 3D DDA traversal.
static TraceResult trace_ray_amanatides_woo(Vector3 ro, Vector3 rd) {
    float t_enter = 0.0f;
    float t_exit = 0.0f;
    // Step 1: clip ray to the voxel grid bounds.
    if (!ray_aabb(ro, rd, &t_enter, &t_exit)) {
        const float sky = clamp_f32(0.5f * (rd.y + 1.0f), 0.0f, 1.0f);
        TraceResult out = {
            .hit = false,
            .entered_grid = false,
            .steps = 0,
            .col = (Vector3){ 0.55f + 0.2f * sky, 0.7f + 0.15f * sky, 0.95f },
        };
        return out;
    }

    // Step 2: start at entry point (or origin if already inside bounds).
    float t = fmaxf(t_enter, 0.0f);
    const Vector3 p = Vector3Add(ro, Vector3Scale(rd, t));

    // Step 3: map start point to initial voxel cell.
    int cell_x = clamp_i32((int) floorf(p.x), 0, GRID_X - 1);
    int cell_y = clamp_i32((int) floorf(p.y), 0, GRID_Y - 1);
    int cell_z = clamp_i32((int) floorf(p.z), 0, GRID_Z - 1);

    // Step 4: determine travel direction (+1 or -1) per axis.
    IVec3 step = { -1, -1, -1 };
    if (rd.x > 0.0f) step.x = 1;
    if (rd.y > 0.0f) step.y = 1;
    if (rd.z > 0.0f) step.z = 1;

    // First boundary crossing candidate on each axis.
    const Vector3 next_boundary = {
        (float) (cell_x + ((step.x > 0) ? 1 : 0)),
        (float) (cell_y + ((step.y > 0) ? 1 : 0)),
        (float) (cell_z + ((step.z > 0) ? 1 : 0))
    };

    const float inf = 1e30f;
    float t_max_x = inf;
    float t_max_y = inf;
    float t_max_z = inf;
    float t_delta_x = inf;
    float t_delta_y = inf;
    float t_delta_z = inf;

    // t_max_*: next crossing along that axis.
    // t_delta_*: crossing distance increment per voxel step on that axis.
    if (fabsf(rd.x) > 1e-6f) {
        t_max_x = t + (next_boundary.x - p.x) / rd.x;
        t_delta_x = fabsf(1.0f / rd.x);
    }
    if (fabsf(rd.y) > 1e-6f) {
        t_max_y = t + (next_boundary.y - p.y) / rd.y;
        t_delta_y = fabsf(1.0f / rd.y);
    }
    if (fabsf(rd.z) > 1e-6f) {
        t_max_z = t + (next_boundary.z - p.z) / rd.z;
        t_delta_z = fabsf(1.0f / rd.z);
    }

    IVec3 normal = { 0, 1, 0 };
    int steps = 0;

    // Core DDA loop: walk voxel-by-voxel along the ray.
    for (int i = 0; i < 256; i++) {
        // Terminate when outside clipped segment or outside grid.
        if (!inside_grid(cell_x, cell_y, cell_z) || (t > t_exit)) {
            break;
        }
        steps += 1;

        // Hit test current voxel.
        const uint8_t id = g_state.voxels[voxel_index(cell_x, cell_y, cell_z)];
        if (id != 0) {
            // Very simple lighting: lambert + height-based ambient term.
            const Vector3 base = sample_voxel_color(id);
            const Vector3 n = { (float) normal.x, (float) normal.y, (float) normal.z };
            const float ndotl = fmaxf(Vector3DotProduct(n, LIGHT_DIR), 0.0f);
            const float ao = 0.7f + 0.3f * ((float) cell_y / (float) GRID_Y);
            TraceResult out = {
                .hit = true,
                .entered_grid = true,
                .steps = steps,
                .col = Vector3Scale(base, 0.2f + 0.8f * ndotl * ao),
            };
            return out;
        }

        // Advance along whichever axis crosses first.
        if ((t_max_x < t_max_y) && (t_max_x < t_max_z)) {
            cell_x += step.x;
            t = t_max_x;
            t_max_x += t_delta_x;
            normal = (IVec3){ -step.x, 0, 0 };
        } else if (t_max_y < t_max_z) {
            cell_y += step.y;
            t = t_max_y;
            t_max_y += t_delta_y;
            normal = (IVec3){ 0, -step.y, 0 };
        } else {
            cell_z += step.z;
            t = t_max_z;
            t_max_z += t_delta_z;
            normal = (IVec3){ 0, 0, -step.z };
        }
    }

    const float sky = clamp_f32(0.5f * (rd.y + 1.0f), 0.0f, 1.0f);
    TraceResult out = {
        .hit = false,
        .entered_grid = true,
        .steps = steps,
        .col = (Vector3){ 0.5f + 0.3f * sky, 0.65f + 0.2f * sky, 0.95f },
    };
    return out;
}

// CPU renderer: one ray per output pixel.
// This is the direct compute-shader candidate if moving traversal to GPU.
static FrameStats render_voxel_image(float dt) {
    FrameStats stats;
    memset(&stats, 0, sizeof(stats));
    stats.rays = IMG_W * IMG_H;

    const Vector3 center = { (float) GRID_X * 0.5f, 3.0f, (float) GRID_Z * 0.5f };
    const float orbit_t = g_state.time_s * 0.6f;
    const float radius = 18.0f;

    // Orbit camera around scene center to make traversal behavior visible.
    Vector3 cam = (Vector3){
        center.x + cosf(orbit_t) * radius,
        8.5f + sinf(orbit_t * 0.7f) * 1.5f,
        center.z + sinf(orbit_t) * radius
    };
    if (g_state.freeze_camera) {
        cam = (Vector3){ center.x + radius, 8.5f, center.z };
    }

    // Build orthonormal camera basis.
    const Vector3 forward = Vector3Normalize(Vector3Subtract(center, cam));
    const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, (Vector3){ 0.0f, 1.0f, 0.0f }));
    const Vector3 up = Vector3Normalize(Vector3CrossProduct(right, forward));

    // Pinhole camera projection constants.
    const float aspect = (float) IMG_W / (float) IMG_H;
    const float fov_scale = tanf((55.0f * 0.5f) * (3.14159265358979323846f / 180.0f));
    const float inv_img_w = 1.0f / (float) IMG_W;
    const float inv_img_h = 1.0f / (float) IMG_H;

    // Incremental ray setup reduces math inside the inner x loop.
    const float u_step = 2.0f * aspect * fov_scale * inv_img_w;
    const float v_step = -2.0f * fov_scale * inv_img_h;
    const float u_start = (-1.0f + inv_img_w) * aspect * fov_scale;
    const float v_start = (1.0f - inv_img_h) * fov_scale;
    const Vector3 ray_step_x = Vector3Scale(right, u_step);

    // Main render loop: trace one ray per output pixel.
    for (int y = 0; y < IMG_H; y++) {
        const float v = v_start + (float) y * v_step;
        const Vector3 row_base = Vector3Add(forward, Vector3Scale(up, v));
        Vector3 ray = Vector3Add(row_base, Vector3Scale(right, u_start));

        int pixel_index = y * IMG_W;
        for (int x = 0; x < IMG_W; x++) {
            const Vector3 dir = Vector3Normalize(ray);
            const TraceResult tr = trace_ray_amanatides_woo(cam, dir);

            if (tr.entered_grid) stats.rays_entered_grid += 1;
            if (tr.hit) stats.hits += 1;
            stats.total_steps += tr.steps;
            if (tr.steps > stats.max_steps) stats.max_steps = tr.steps;

            const int r = clamp_i32((int) (tr.col.x * 255.0f), 0, 255);
            const int g = clamp_i32((int) (tr.col.y * 255.0f), 0, 255);
            const int b = clamp_i32((int) (tr.col.z * 255.0f), 0, 255);
            // Store shaded color in CPU image buffer.
            g_state.pixels[pixel_index++] = (Color){
                (unsigned char) r,
                (unsigned char) g,
                (unsigned char) b,
                255
            };

            ray = Vector3Add(ray, ray_step_x);
        }
    }

    if (stats.rays > 0) {
        stats.avg_steps_per_ray = (float) stats.total_steps / (float) stats.rays;
        stats.hit_ratio = (float) stats.hits / (float) stats.rays;
    }
    if (dt > 1e-6f) {
        stats.rays_per_sec = (float) stats.rays / dt;
        stats.steps_per_sec = (float) stats.total_steps / dt;
    }

    return stats;
}

static inline int ui_font_size(void) {
    const int fs = (int) lroundf(18.0f * UI_FONT_SCALE);
    return (fs > 8) ? fs : 8;
}

static inline int ui_line_height(void) {
    return ui_font_size() + (int) lroundf(6.0f * UI_FONT_SCALE);
}

static bool ui_button(Rectangle rect, const char* label) {
    const Vector2 m = GetMousePosition();
    const bool hover = CheckCollisionPointRec(m, rect);
    const bool pressed = hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    const Color bg = hover ? (Color){ 100, 106, 118, 235 } : (Color){ 76, 80, 90, 235 };
    DrawRectangleRec(rect, bg);
    DrawRectangleLinesEx(rect, 1.0f, (Color){ 32, 34, 40, 255 });

    const int fs = ui_font_size();
    const int tw = MeasureText(label, fs);
    DrawText(
        label,
        (int) (rect.x + (rect.width - (float) tw) * 0.5f),
        (int) (rect.y + (rect.height - (float) fs) * 0.5f),
        fs,
        (Color){ 245, 245, 245, 255 }
    );

    return pressed;
}

// Runtime diagnostics and controls drawn over final image.
static void draw_overlay(void) {
    const int line_h = ui_line_height();
    const int pad = (int) lroundf(10.0f * UI_FONT_SCALE);
    const int x = 12;
    const int y = 12;
    const int w = (int) lroundf(560.0f * UI_FONT_SCALE);

    const int fs = ui_font_size();
    const int button_h = fs + (int) lroundf(12.0f * UI_FONT_SCALE);

    int row = 0;
    row += 11; // text rows
    row += 1;  // button row
    const int h = pad * 2 + row * line_h + button_h;

    DrawRectangle(x, y, w, h, (Color){ 28, 30, 36, 225 });
    DrawRectangleLines(x, y, w, h, (Color){ 16, 18, 22, 255 });

    int ty = y + pad;
    const int tx = x + pad;

    DrawText(TextFormat("Technique: Fast Voxel Traversal (3D DDA)"), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Grid: %dx%dx%d voxels", GRID_X, GRID_Y, GRID_Z), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Ray buffer: %dx%d (%d rays/frame)", IMG_W, IMG_H, g_state.frame_stats.rays), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Camera: %s", g_state.freeze_camera ? "frozen" : "orbiting"), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("DDA: AABB entry -> tMax/tDelta stepping per axis"), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Exit: first solid voxel, grid boundary, or 256 steps"), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Frame: %.2f ms | FPS(avg): %.1f", g_state.frame_ms, g_state.fps_smooth), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Rays/s: %.2f M | Steps/s: %.2f M", g_state.frame_stats.rays_per_sec / 1000000.0f, g_state.frame_stats.steps_per_sec / 1000000.0f), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("AABB entered: %d / %d", g_state.frame_stats.rays_entered_grid, g_state.frame_stats.rays), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Hits: %d (%.1f%%)", g_state.frame_stats.hits, g_state.frame_stats.hit_ratio * 100.0f), tx, ty, fs, RAYWHITE); ty += line_h;
    DrawText(TextFormat("Traversal steps: avg %.2f | max %d", g_state.frame_stats.avg_steps_per_ray, g_state.frame_stats.max_steps), tx, ty, fs, RAYWHITE); ty += line_h;

    const float btn_y = (float) (ty + (int) lroundf(2.0f * UI_FONT_SCALE));
    const float btn_w = (float) ((w - pad * 3) / 2);
    const Rectangle btn_freeze = { (float) tx, btn_y, btn_w, (float) button_h };
    const Rectangle btn_quit = { (float) tx + btn_w + (float) pad, btn_y, btn_w, (float) button_h };

    if (ui_button(btn_freeze, g_state.freeze_camera ? "Resume Camera" : "Freeze Camera")) {
        g_state.freeze_camera = !g_state.freeze_camera;
    }
    if (ui_button(btn_quit, "Quit")) {
        g_state.request_quit = true;
    }
}

int main(void) {
    // 1) Initialize window and target framerate.
    InitWindow(1280, 720, "C + raylib + Amanatides-Woo");
    SetTargetFPS(60);

    // 2) Build scene and initialize CPU/GPU image resources.
    build_scene();
    memset(g_state.pixels, 0, sizeof(g_state.pixels));

    Image img = GenImageColor(IMG_W, IMG_H, BLACK);
    g_state.ray_texture = LoadTextureFromImage(img);
    UnloadImage(img);
    SetTextureFilter(g_state.ray_texture, TEXTURE_FILTER_POINT);

    // 3) Standard raylib frame loop.
    while (!WindowShouldClose() && !g_state.request_quit) {
        const float dt = clamp_f32(GetFrameTime(), 1e-5f, 0.25f);

        // Update camera timer (unless frozen from UI).
        if (!g_state.freeze_camera) {
            g_state.time_s += dt;
        }

        // Smooth fps readout for overlay stability.
        g_state.frame_ms = dt * 1000.0f;
        const float fps = (dt > 1e-6f) ? (1.0f / dt) : 0.0f;
        if (g_state.fps_smooth <= 0.0f) {
            g_state.fps_smooth = fps;
        } else {
            g_state.fps_smooth = g_state.fps_smooth * 0.9f + fps * 0.1f;
        }

        // Raytrace on CPU, then upload texture for presentation.
        g_state.frame_stats = render_voxel_image(dt);
        UpdateTexture(g_state.ray_texture, g_state.pixels);

        // Draw full-screen image and overlay UI.
        BeginDrawing();
        ClearBackground((Color){ 20, 20, 26, 255 });

        DrawTexturePro(
            g_state.ray_texture,
            (Rectangle){ 0.0f, 0.0f, (float) IMG_W, (float) IMG_H },
            (Rectangle){ 0.0f, 0.0f, (float) GetScreenWidth(), (float) GetScreenHeight() },
            (Vector2){ 0.0f, 0.0f },
            0.0f,
            WHITE
        );

        draw_overlay();
        EndDrawing();
    }

    // 4) Release resources.
    UnloadTexture(g_state.ray_texture);
    CloseWindow();
    return 0;
}
