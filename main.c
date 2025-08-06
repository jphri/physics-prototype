#include <stdio.h>
#include <stdlib.h>
#include <SDL2/SDL.h>
#include <assert.h>
#include <stdint.h>

#include "util.h"
#include "measure.h"

#define N_BODY 2048
#define PHYSICS_ITERATIONS (8 * 60)
#define PHYSICS_TIME (1.0 / PHYSICS_ITERATIONS)

#define GRID_SIZE 100
#define GRID_BUFFER_SIZE (GRID_SIZE * GRID_SIZE)
#define GRID_TILE_SIZE 32

#define UINTMAX_BITS (sizeof(uintmax_t) * 8)

typedef float Float;
typedef struct {
	Float position[2];
	Float velocity[2];
	Float acceleration[2];
	Float half_size[2];
	
	Float mass;
	Float restitution;
	int is_static;
} Body;

typedef struct {
	int next, prev;
	Body *body;
} BodyNodeList;

#define RAND_FLOAT (rand() / (Float)RAND_MAX)
#define RAND(MIN, MAX) (RAND_FLOAT * (MAX - MIN) + MIN)

static void solve_body(Body *body, Float delta);
static void solve_body_grid_list(int grid_list, int other_grid, Float delta);
static void solve_body_grid_list_static(int grid_list, int other_grid, Float delta);
static void solve_body_grid(int body_node, int other_grid, Float delta);
static void solve_body_grid_static(int body_node, int other_grid, Float delta);
static void update_body(Body *body, Float delta);
static void render_body(Body *body);

static BodyNodeList *blist(int id);
static void          add_body_list(int *body_list, Body *);
static void          clear_lists();
static void          calculate_grid();
static void          calculate_grid_body(Body *);

static void test_and_solve(Body *body, Body *body2, Float delta);
static void test_and_solve_static(Body *body, Body *body2, Float delta);

static SDL_Window *window;
static SDL_Renderer *renderer;
static Body body_list[N_BODY];
static ArrayBuffer body_node_buffer, body_already_checked_buffer;
static int body_count;

static int grid_list[GRID_BUFFER_SIZE];
static int static_grid_list[GRID_BUFFER_SIZE];

static int max_object_count = 0;
static int object_count = 0;
static int object_sum = 0;
static int iterations = 0;
static int count_20 = 0;

static int already_checked(int body1_id, int body2_id)
{
	unsigned int element_size = (body_count + UINTMAX_BITS) / UINTMAX_BITS;

	unsigned int body1_check_offset = body1_id * element_size;
	unsigned int body2_element = body2_id / UINTMAX_BITS;
	unsigned int body2_bit = body2_id % UINTMAX_BITS;
	uintmax_t element = *((uintmax_t*)body_already_checked_buffer.data) + body1_check_offset + body2_element;

	return (element & (1 << body2_bit)) != 0;
}

static void mark_checked(int body1_id, int body2_id)
{
	unsigned int element_size = (body_count + UINTMAX_BITS) / UINTMAX_BITS;
	unsigned int body1_check_offset = body1_id * element_size;
	unsigned int body2_element = body2_id / UINTMAX_BITS;
	unsigned int body2_bit = body2_id % UINTMAX_BITS;
	uintmax_t *element = ((uintmax_t*)body_already_checked_buffer.data) + body1_check_offset + body2_element;

	*element |= (1 << body2_bit);
}

int
main()
{
	SDL_Init(SDL_INIT_VIDEO);
	window = SDL_CreateWindow("hello",
			SDL_WINDOWPOS_CENTERED,
			SDL_WINDOWPOS_CENTERED,
			800, 600,
			SDL_WINDOW_OPENGL);
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

	arrbuf_init(&body_node_buffer);
	arrbuf_init(&body_already_checked_buffer);
	clear_lists();

	body_count = 5;
	body_list[0].position[0] = 400;
	body_list[0].position[1] = 500;
	body_list[0].half_size[0] = 100;
	body_list[0].half_size[1] = 5;
	body_list[0].velocity[0] = 0;
	body_list[0].velocity[1] = 0;
	body_list[0].acceleration[0] = 0;
	body_list[0].acceleration[1] = 0;
	body_list[0].is_static = 1;
	body_list[0].restitution = 0.5;
	body_list[0].mass = 10.0;

	body_list[1].position[0] = 500;
	body_list[1].position[1] = 400;
	body_list[1].half_size[0] = 5;
	body_list[1].half_size[1] = 100;
	body_list[1].velocity[0] = 0;
	body_list[1].velocity[1] = 0;
	body_list[1].acceleration[0] = 0;
	body_list[1].acceleration[1] = 0;
	body_list[1].is_static = 1;
	body_list[1].restitution = 0.5;
	body_list[1].mass = 10.0;

	body_list[2].position[0] = 400;
	body_list[2].position[1] = 550;
	body_list[2].half_size[0] = 400;
	body_list[2].half_size[1] = 10;
	body_list[2].velocity[0] = 0;
	body_list[2].velocity[1] = 0;
	body_list[2].acceleration[0] = 0;
	body_list[2].acceleration[1] = 0;
	body_list[2].is_static = 1;
	body_list[2].restitution = 0.5;
	body_list[2].mass = 10.0;

	body_list[3].position[0] = 5;
	body_list[3].position[1] = 300;
	body_list[3].half_size[0] = 10;
	body_list[3].half_size[1] = 300;
	body_list[3].velocity[0] = 0;
	body_list[3].velocity[1] = 0;
	body_list[3].acceleration[0] = 0;
	body_list[3].acceleration[1] = 0;
	body_list[3].is_static = 1;
	body_list[3].restitution = 0.5;
	body_list[3].mass = 10.0;

	body_list[4].position[0] = 750;
	body_list[4].position[1] = 300;
	body_list[4].half_size[0] = 10;
	body_list[4].half_size[1] = 300;
	body_list[4].velocity[0] = 0;
	body_list[4].velocity[1] = 0;
	body_list[4].acceleration[0] = 0;
	body_list[4].acceleration[1] = 0;
	body_list[4].is_static = 1;
	body_list[4].restitution = 0.5;
	body_list[4].mass = 10.0;

	Uint64 prev_time = SDL_GetPerformanceCounter();
	static float physics_time = 0;
	static float add_time = 0;
	static float fps_time = 0, physics_time_avg;
	static int frames = 0;
	for(;;) {
		Uint64 curr_time = SDL_GetPerformanceCounter();
		Float delta = (Float)(curr_time - prev_time) / SDL_GetPerformanceFrequency();
		prev_time = curr_time;
		if(delta > 0.25)
			delta = 0.25;
		
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);
		
		physics_time += delta;
		static int physics_count = 0;
		if(physics_time > PHYSICS_TIME) {
			Uint64 start = SDL_GetPerformanceCounter();
			while(physics_time > PHYSICS_TIME) {
				iterations++;
				int count = 0;
				arrbuf_clear(&body_already_checked_buffer);
				count = ((body_count + UINTMAX_BITS) / UINTMAX_BITS) * body_count;

				for(int i = 0; i < count; i++)
					*(uintmax_t*)(arrbuf_newptr(&body_already_checked_buffer, sizeof(uintmax_t))) = 0;

				calculate_grid();
				max_object_count = 0;
				for(int i = 0; i < GRID_BUFFER_SIZE; i++) {
					int x = i % GRID_SIZE;
					int y = i / GRID_SIZE;

					object_count = 0;

					solve_body_grid_list(grid_list[x + y * GRID_SIZE], grid_list[x + y * GRID_SIZE], delta);
					solve_body_grid_list_static(grid_list[x + y * GRID_SIZE], static_grid_list[x + y * GRID_SIZE], delta);
					if(object_count > max_object_count)
						max_object_count = object_count;

					object_sum += object_count;
					if(object_count > 20)
						count_20 ++;
				}
				for(int i = 0; i < body_count; i++)
					update_body(&body_list[i], PHYSICS_TIME);

				physics_time -= PHYSICS_TIME;
				physics_count ++;
				static int flip = 0;
				if(physics_count > PHYSICS_ITERATIONS * 0.005) {
					flip = (flip + 1) % 2;
					if(body_count < N_BODY) {
						body_list[body_count].half_size[0] = RAND(2, 5);
						body_list[body_count].half_size[1] = RAND(2, 5);
						body_list[body_count].acceleration[0] = 0;
						body_list[body_count].acceleration[1] = 0;
						body_list[body_count].position[0] = 50 + flip * 500;
						body_list[body_count].position[1] = 50;
						body_list[body_count].velocity[0] = RAND(0.0, 200.0) * -(flip * 2 - 1);
						body_list[body_count].velocity[1] = 0.0;
						body_list[body_count].mass = RAND(5, 10);
						body_list[body_count].restitution = RAND(0.0, 0.5);
						body_count ++;
					}
					physics_count = 0;
				}
			}
			Uint64 end = SDL_GetPerformanceCounter();
			physics_time_avg += 1000.0 * (end - start) / SDL_GetPerformanceFrequency();
		}

		frames++;
		fps_time += delta; 
		if(fps_time > 1.0) {
			printf("FPS: %d | SYM_TIME: %f | BODY_COUNT: %d | MAX: %d | AVG: %f | 20: %f\n",
					frames,
					physics_time_avg / frames,
					body_count, 
					max_object_count, 
					object_sum / (float)(iterations * GRID_BUFFER_SIZE), 
					(float)count_20 / iterations);

			count_20 = 0;
			object_sum = 0;
			frames = 0;
			physics_time_avg = 0;
			fps_time = 0;
			iterations = 0;
		}

		for(int i = 0; i < body_count; i++)
			render_body(&body_list[i]);

		SDL_RenderPresent(renderer);
		SDL_Event event;
		while(SDL_PollEvent(&event)) {
			switch(event.type) {
			case SDL_QUIT:
				goto end_game;
				break;
			}
		}
	}

end_game:
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}

int
check_collision(Body *body, Body *body2, Float delta, Float hit_position[2], Float hit_normal[2], Float pen_vector[2])
{
	Float total_hs[2], velocity[2];
	Float dt[2], ht[2];

	Float first_exit = INFINITY, last_entry = -INFINITY;
	total_hs[0] = body->half_size[0] + body2->half_size[0];
	total_hs[1] = body->half_size[1] + body2->half_size[1];
	velocity[0] = body2->velocity[0] - body->velocity[0];
	velocity[1] = body2->velocity[1] - body->velocity[1];

	int check = 
		(body->position[0] < body2->position[0] - total_hs[0]) + (body->position[0] > body2->position[0] + total_hs[0]) +
		(body->position[1] < body2->position[1] - total_hs[1]) + (body->position[1] > body2->position[1] + total_hs[1]);

	if(check)
		return 0;

	dt[0] = body2->position[0] - body->position[0];
	dt[1] = body2->position[1] - body->position[1];
	ht[0] = total_hs[0] - fabsf(dt[0]);
	ht[1] = total_hs[1] - fabsf(dt[1]);
	hit_normal[0] = (ht[0] < ht[1]) * ((dt[0] > 0) - (dt[0] < 0));
	hit_normal[1] = (ht[0] > ht[1]) * ((dt[1] > 0) - (dt[1] < 0));
	pen_vector[0] = ht[0] * hit_normal[0];
	pen_vector[1] = ht[1] * hit_normal[1];
	hit_position[0] = body->position[0] - pen_vector[0];
	hit_position[1] = body->position[1] - pen_vector[1];

	return 1;
}

void
solve_body_grid_list(int grid_list, int grid_other, Float delta)
{
	while(grid_list >= 0) {
		int other = blist(grid_list)->next;

		while(other >= 0) {
			solve_body_grid(grid_list, other, delta);
			other = blist(other)->next;
		}

		grid_list = blist(grid_list)->next;
		object_count++;
	}
}

void
solve_body_grid_list_static(int grid_list, int grid_other, Float delta)
{
	while(grid_list >= 0) {
		int other = grid_other;

		while(other >= 0) {
			solve_body_grid_static(grid_list, other, delta);
			other = blist(other)->next;
		}
		grid_list = blist(grid_list)->next;
		object_count++;
	}
}

void
solve_body_grid(int body_node, int other_body, Float delta)
{
	Body *body = blist(body_node)->body;
	Body *body2 = blist(other_body)->body;
	test_and_solve(body, body2, delta);
}

void
solve_body_grid_static(int body_node, int other_body, Float delta)
{
	Body *body = blist(body_node)->body;
	Body *body2 = blist(other_body)->body;
	test_and_solve_static(body, body2, delta);
}

void
solve_body(Body *body, Float delta) 
{
	for(int i = (int)(body - body_list) + 1; i < body_count; i++) {
		Float position[2], normal[2], pen_vector[2];
		if(check_collision(body, &body_list[i], delta, position, normal, pen_vector)) {
			float j;
			Float relative_vel[2];

			relative_vel[0] = body->velocity[0] - body_list[i].velocity[0];
			relative_vel[1] = body->velocity[1] - body_list[i].velocity[1];
			j = relative_vel[0] * normal[0] + relative_vel[1] * normal[1];
			j *= -(1 + 0.5);
			j /= 2.0;

			body->position[0] -= pen_vector[0] * 0.5;
			body->position[1] -= pen_vector[1] * 0.5;
			body_list[i].position[0] += pen_vector[0] * 0.5;
			body_list[i].position[1] += pen_vector[1] * 0.5;

			body->velocity[0]        += normal[0] * (j);
			body->velocity[1]        += normal[1] * (j);
			body_list[i].velocity[0] -= normal[0] * (j);
			body_list[i].velocity[1] -= normal[1] * (j);
		}
	}

	if(body->position[0] < 0) {
		body->velocity[0] = 0.0;
		body->acceleration[0] = 0.0;
		body->position[0] = 0;
	}

	if(body->position[1] < 0) {
		body->velocity[1] = 0.0;
		body->acceleration[1] = 0.0;
		body->position[1] = 0;
	}

	if(body->position[0] > 800 - body->half_size[0]) {
		body->velocity[0] = 0.0;
		body->acceleration[0] = 0.0;
		body->position[0] = 800 - body->half_size[0];
	}

	if(body->position[1] > 600 - body->half_size[1]) {
		body->velocity[1] = 0.0;
		body->acceleration[1] = 0.0;
		body->position[1] = 600 - body->half_size[1];
	}
}

void
update_body(Body *body, Float delta) 
{
	Float velocity[2];

	if(!body->is_static) {
		body->acceleration[1] = 19.4 * 4;
		body->velocity[0] += body->acceleration[0] * delta;
		body->velocity[1] += body->acceleration[1] * delta;
		body->position[0] += body->velocity[0] * delta;
		body->position[1] += body->velocity[1] * delta;
		body->acceleration[0] = 0;
		body->acceleration[1] = 0;
	}
}

void
render_body(Body *body) 
{
	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
	SDL_RenderDrawRect(renderer, &(SDL_Rect){
		.x = body->position[0] - body->half_size[0],
		.y = body->position[1] - body->half_size[1],
		.w = body->half_size[0] * 2.0,
		.h = body->half_size[1] * 2.0
	});
}

static void
add_body_list(int *body_list, Body *b) 
{
	BodyNodeList *new = arrbuf_newptr(&body_node_buffer, sizeof(BodyNodeList));
	int id =  (int)(new - (BodyNodeList*)body_node_buffer.data);

	new->body = b;
	new->next = *body_list;

	if(*body_list >= 0)
		blist(*body_list)->prev = id;

	new->prev = -1;
	*body_list = id;
}

static BodyNodeList *
blist(int id)
{
	if(id >= 0)
		return &((BodyNodeList*)body_node_buffer.data)[id];
	assert(0 && "YOU ARE USING SOMETHING THAT IS ZERO, RETARDED!");
}

static void 
clear_lists()
{
	for(int i = 0; i < GRID_BUFFER_SIZE; i++) {
		grid_list[i] = -1;
		static_grid_list[i] = -1;
	}
	arrbuf_clear(&body_node_buffer);
}

static void
calculate_grid()
{
	clear_lists();
	for(int i = 0; i < body_count; i++)
		calculate_grid_body(&body_list[i]);
}

static void
calculate_grid_body(Body *b)
{
	int x, x_max;
	int y, y_max;
	int count;
	
	x     = floorf((b->position[0] - b->half_size[0]) / GRID_TILE_SIZE);
	x_max = floorf((b->position[0] + b->half_size[0]) / GRID_TILE_SIZE);
	y_max = floorf((b->position[1] + b->half_size[1]) / GRID_TILE_SIZE);
	for(; x <= x_max; x++) {
		if(x < 0 || x >= GRID_SIZE)
			continue;

		y = floorf((b->position[1] - b->half_size[1]) / GRID_TILE_SIZE);
		for(; y <= y_max; y++) {

			if(y < 0 || y >= GRID_SIZE)
				continue;

			if(b->is_static)
				add_body_list(&static_grid_list[x + y * GRID_SIZE], b);
			else
				add_body_list(&grid_list[x + y * GRID_SIZE], b);
		}
	}
}

static void
test_and_solve(Body *body, Body *body2, Float delta)
{
	Float position[2], normal[2], pen_vector[2];
	
	if(check_collision(body, body2, delta, position, normal, pen_vector)) {
		float j;
		Float relative_vel[2];
		
		Float inertia_1 = 1.0 / body->mass;
		Float inertia_2 = 1.0 / body2->mass;
		Float total_mass = body->mass + body2->mass;
		
		relative_vel[0] = body->velocity[0] - body2->velocity[0];
		relative_vel[1] = body->velocity[1] - body2->velocity[1];
		j = relative_vel[0] * normal[0] + relative_vel[1] * normal[1];
		j *= -(1 + body->restitution + body2->restitution);
		j /= (inertia_1 + inertia_2);

		body->position[0] -= pen_vector[0] * (body2->is_static ? 1.0 : body->mass / total_mass);
		body->position[1] -= pen_vector[1] * (body2->is_static ? 1.0 : body->mass / total_mass);
		body->velocity[0] += normal[0] * (j * inertia_1);
		body->velocity[1] += normal[1] * (j * inertia_1);

		body2->position[0] += pen_vector[0] * (body->is_static ? 1.0 : body2->mass / total_mass);
		body2->position[1] += pen_vector[1] * (body->is_static ? 1.0 : body2->mass / total_mass);
		body2->velocity[0] -= normal[0] * (j * inertia_2);
		body2->velocity[1] -= normal[1] * (j * inertia_2);
	}
}

static void
test_and_solve_static(Body *body, Body *stat, Float delta)
{
	Float position[2], normal[2], pen_vector[2];
	
	if(check_collision(body, stat, delta, position, normal, pen_vector)) {
		float j;
		Float relative_vel[2];
		
		Float inertia_1 = body->is_static ? 0 : 1.0 / body->mass;
		Float total_mass = body->mass;
		
		relative_vel[0] = body->velocity[0];
		relative_vel[1] = body->velocity[1];
		j = relative_vel[0] * normal[0] + relative_vel[1] * normal[1];
		j *= -(1 + body->restitution + stat->restitution);
		j /= (inertia_1);

		body->position[0] -= pen_vector[0];
		body->position[1] -= pen_vector[1];
		body->velocity[0] += normal[0] * (j * inertia_1);
		body->velocity[1] += normal[1] * (j * inertia_1);
	}
}
