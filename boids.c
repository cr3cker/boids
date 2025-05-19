#include <math.h>
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080
#define N 800

float avoid_factor = 1.0f;
float centering_factor = 1.0f;
float matching_factor = 1.0f;
float visual_range = 75.0f;
float protected_range = 20.0f;

typedef struct {
    Vector2 pos;
    Vector2 vel;
    Vector2 v1, v2, v3;
    float angle_deg;
    float size;
} Boid;

typedef struct {
    Boid **data;
    int count;
    int capacity;
} BoidList;

typedef struct {
    Vector2 pos_sum;
    int count;
} CohesionData;

typedef struct {
    Boid *self;
    Vector2 repulsion;
} SeparationData;

typedef struct {
    Boid *self;
    Vector2 vel_sum;
    int count;
} AlignmentData;

BoidList **grid = NULL;
int rows, cols;

void init_grid(int screen_width, int screen_height, float visual_range) {
    rows = (int)ceilf(screen_height / visual_range);
    cols = (int)ceilf(screen_width / visual_range);

    grid = malloc(rows * sizeof(BoidList*));
    for (int i = 0; i < rows; i++) {
        grid[i] = malloc(cols * sizeof(BoidList));

        for (int j = 0; j < cols; j++) {
            grid[i][j].count = 0;
            grid[i][j].data = NULL;
            grid[i][j].capacity = 0;
        }
    }
}

void free_grid() {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            free(grid[i][j].data);
        }
        free(grid[i]);
    }
    free(grid);
    grid = NULL;
}

void boidlist_add(BoidList *list, Boid *b) {
    if (list->count == list->capacity) {
        int new_capacity = list->capacity == 0 ? 4 : list->capacity * 2;
        list->data = realloc(list->data, new_capacity * sizeof(Boid*));
        list->capacity = new_capacity;
    }
    list->data[list->count++] = b;
}

void add_boid_to_grid(Boid *b) {
    int row = (int)(b->pos.y / visual_range);
    int col = (int)(b->pos.x / visual_range);

    if (row < 0) row = 0;
    if (row >= rows) row = rows - 1;
    if (col < 0) col = 0;
    if (col >= cols) col = cols - 1;

    boidlist_add(&grid[row][col], b);
}

void clear_grid() {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            free(grid[i][j].data);
            grid[i][j].count = 0;
            grid[i][j].capacity = 0;
            grid[i][j].data = NULL;
        }
    }
}

void for_neighbors(Boid *b, void (*callback)(Boid *neighbor, void *user_data), void *user_data) {
    int row = (int)(b->pos.y / visual_range);
    int col = (int)(b->pos.x / visual_range);

    for (int i = row - 1; i <= row + 1; i++) {
        if (i < 0 || i >= rows) continue;
        for (int j = col - 1; j <= col + 1; j++) {
            if (j < 0 || j >= cols) continue;
            BoidList *cell = &grid[i][j];
            for (int k = 0; k < cell->count; k++) {
                Boid *neighbor = cell->data[k];
                if (neighbor != b) callback(neighbor, user_data);
            }
        }
    }
}

Vector2 vector2divide(Vector2 v1, float v) {
    return (Vector2){ v1.x / v, v1.y / v };
}

void cohesion_callback(Boid *neighbor, void *user_data) {
    CohesionData *data = (CohesionData *)user_data;
    data->pos_sum = Vector2Add(data->pos_sum, neighbor->pos);
    data->count++;
}

Vector2 cohesion(Boid *b) {
    CohesionData data = { Vector2Zero(), 0 };
    for_neighbors(b, cohesion_callback, &data);

    if (data.count == 0) return Vector2Zero();
    Vector2 avg = vector2divide(data.pos_sum, data.count);
    
    return Vector2Subtract(avg, b->pos);
}

void separation_callback(Boid *neighbor, void *user_data) {
    SeparationData *data = (SeparationData *)user_data;
    
    if (Vector2Equals(data->self->pos, neighbor->pos)) return;
    float dist_sq = Vector2DistanceSqr(data->self->pos, neighbor->pos);
    if (dist_sq < protected_range * protected_range) {
        Vector2 diff = Vector2Subtract(data->self->pos, neighbor->pos);
        data->repulsion = Vector2Add(data->repulsion, diff);
    }
}

Vector2 separation(Boid *b) {
    SeparationData data = { b, Vector2Zero() };
    for_neighbors(b, separation_callback, &data);
    return data.repulsion;
}

void alignment_callback(Boid *neighbor, void *user_data) {
    AlignmentData *data = (AlignmentData *)user_data;
    
    if (Vector2Equals(data->self->pos, neighbor->pos)) return;
    float dist = Vector2Distance(data->self->pos, neighbor->pos);
    if (dist < visual_range) {
        data->vel_sum = Vector2Add(data->vel_sum, neighbor->vel);
        data->count++;
    }
}

Vector2 alignment(Boid *b) {
    AlignmentData data = { b, Vector2Zero(), 0 };
    for_neighbors(b, alignment_callback, &data);
    
    if (data.count == 0) return Vector2Zero();
    data.vel_sum = vector2divide(data.vel_sum, data.count);

    return Vector2Subtract(data.vel_sum, b->vel);
}

float lerp_angle(float a, float b, float t) {
    float diff = fmodf(b - a + 360.0f, 360.0f);
    if (diff > 180.0f) diff -= 360;
    return fmodf(a + diff * t + 360.0f, 360.0f);
}

Boid new_boid() {
    Vector2 pos = { GetRandomValue(0, GetScreenWidth()), GetRandomValue(0, GetScreenHeight()) };
    int sign_x = GetRandomValue(0, 1) ? 1 : -1;
    int sign_y = GetRandomValue(0, 1) ? 1 : -1;
    Vector2 vel = { GetRandomValue(80, 160) * sign_x , GetRandomValue(80, 160) * sign_y };
    float size = 15;
    Boid b = {
        .pos = pos,
        .vel = vel,
        .angle_deg = 0.0f,
        .size = size
    };
    return b;
}

void bound_position(Boid *b) {
    float margin = 100.0f;
    float turn_strengh = 80.0f;

    if (b->pos.x < margin) b->vel.x += (margin - b->pos.x) / margin * turn_strengh;
    else if (b->pos.x > SCREEN_WIDTH - margin) b->vel.x -= (b->pos.x - (SCREEN_WIDTH - margin)) / margin * turn_strengh;
    if (b->pos.y < margin) b->vel.y += (margin - b->pos.y) / margin * turn_strengh;
    else if (b->pos.y > SCREEN_HEIGHT - margin) b->vel.y -= (b->pos.y - (SCREEN_HEIGHT - margin)) / margin * turn_strengh;
}

void rotate_point(Vector2 *point, Vector2 center, float angle_deg) {
    float rad = angle_deg * DEG2RAD;
    Vector2 rel = Vector2Subtract(*point, center);
    float sin_a = sinf(rad);
    float cos_a = cosf(rad); 
    Vector2 rotated = {
        cos_a * rel.x - rel.y * sin_a,
        sin_a * rel.x + rel.y * cos_a
    };
    *point = Vector2Add(rotated, center);
}

void update_boid(Boid *b, float dt) {
    Vector2 separation_force = Vector2Scale(separation(b), avoid_factor);
    Vector2 cohesion_force = Vector2Scale(cohesion(b), centering_factor);
    Vector2 alignment_force = Vector2Scale(alignment(b), matching_factor);
   
    Vector2 desired_vel = b->vel;
    desired_vel = Vector2Add(desired_vel, alignment_force);
    desired_vel = Vector2Add(desired_vel, cohesion_force);
    desired_vel = Vector2Add(desired_vel, separation_force);

    b->vel = Vector2Lerp(b->vel, desired_vel, 0.1f);

    float max_speed = 400.0f;
    if (Vector2Length(b->vel) > max_speed) {
        b->vel = Vector2Scale(Vector2Normalize(b->vel), max_speed);
    }

    b->pos = Vector2Add(b->pos, Vector2Scale(b->vel, dt));
    bound_position(b);
    float target_angle = atan2f(b->vel.y, b->vel.x) * RAD2DEG;

    b->angle_deg = lerp_angle(b->angle_deg, target_angle, 3.0f * dt);

    float s = b->size;
    b->v1 = (Vector2){ b->pos.x + s / 2.0f, b->pos.y };
    b->v2 = (Vector2){ b->pos.x - s / 2.0f, b->pos.y - s / 3.0f };
    b->v3 = (Vector2){ b->pos.x - s / 2.0f, b->pos.y + s / 3.0f };

    rotate_point(&b->v1, b->pos, b->angle_deg);
    rotate_point(&b->v2, b->pos, b->angle_deg);
    rotate_point(&b->v3, b->pos, b->angle_deg);
}

void draw_boid(Boid b) {
    DrawTriangle(b.v1, b.v2, b.v3, RED);
}

void draw_borders() {
    DrawLineV((Vector2){0, 0}, (Vector2){SCREEN_WIDTH, 0}, BLACK);
    DrawLineV((Vector2){0, 0}, (Vector2){0, SCREEN_HEIGHT}, BLACK);
    DrawLineV((Vector2){SCREEN_WIDTH, 0}, (Vector2){SCREEN_WIDTH, SCREEN_HEIGHT}, BLACK);
    DrawLineV((Vector2){0, SCREEN_HEIGHT}, (Vector2){SCREEN_WIDTH, SCREEN_HEIGHT}, BLACK);
}

void handle_controls(Camera2D *camera) {
    camera->zoom = expf(logf(camera->zoom) + ((float)GetMouseWheelMove()*0.1f));

    if (camera->zoom > 3.0f) camera->zoom = 3.0f;
    else if (camera->zoom < 0.1f) camera->zoom = 0.1f;

    if (IsKeyPressed(KEY_R)) {
        camera->zoom = 0.8f;
        camera->offset = (Vector2){ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    }

    if (IsKeyDown(KEY_W)) camera->offset.y += 10;
    if (IsKeyDown(KEY_S)) camera->offset.y -= 10;
    if (IsKeyDown(KEY_A)) camera->offset.x += 10;
    if (IsKeyDown(KEY_D)) camera->offset.x -= 10;
}

void draw_controls() {
    int x = 100;
    int y = 20;
    int spacing = 30;

    GuiSliderBar((Rectangle){ x, y, 200, 20 }, "Avoid Factor", TextFormat("%.2f", avoid_factor), &avoid_factor, 0.0f, 5.0f);
    y += spacing;
    GuiSliderBar((Rectangle){ x, y, 200, 20 }, "Centering Factor", TextFormat("%.2f", centering_factor), &centering_factor, 0.0f, 5.0f);
    y += spacing;
    GuiSliderBar((Rectangle){ x, y, 200, 20 }, "Matching Factor", TextFormat("%.2f", matching_factor), &matching_factor, 0.0f, 5.0f);
    y += spacing;
    GuiSliderBar((Rectangle){ x, y, 200, 20 }, "Visual Range", TextFormat("%.0f", visual_range), &visual_range, 10.0f, 300.0f);
    y += spacing;
    GuiSliderBar((Rectangle){ x, y, 200, 20 }, "Protected Range", TextFormat("%.0f", protected_range), &protected_range, 5.0f, 100.0f);
}

int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Boids Simulation");

    Boid *boids = (Boid *)malloc(N * sizeof(Boid));

    for (int i = 0; i < N; i++) {
        boids[i] = new_boid();
    }

    Camera2D camera = { 0 };
    camera.zoom = 0.8f;
    camera.target = (Vector2){ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    camera.offset = (Vector2){ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };

    init_grid(SCREEN_WIDTH, SCREEN_HEIGHT, visual_range);
    
    while (!WindowShouldClose()) {
        clear_grid();
        for (int i = 0; i < N; i++) {
            add_boid_to_grid(&boids[i]);
        }
        float dt = GetFrameTime();

        handle_controls(&camera);

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode2D(camera);

        for (int i = 0; i < N; i++) {
            update_boid(&boids[i], dt);
            draw_boid(boids[i]);
        }

        draw_borders();

        EndMode2D();
        draw_controls();
        EndDrawing();
    }
    free(boids);
    free_grid();
    CloseWindow();
    return 0;
}
