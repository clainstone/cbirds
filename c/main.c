#include "boids.h"
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define _XOPEN_SOURCE 600
#define ROTATION_FRAME 360
#define FRAME_ANGLE (360 / ROTATION_FRAME)
#define BIRDS_NUM 1600
#define BIRD_WIDTH 20
#define BIRD_HEIGTH 20
#define PNG_FORMAT 100
#define FRAME_RATE 144
#define PERIOD_MULTIPL 1000000
#define DEF_TERMINAL_WIDTH 100
#define DEF_TERMINAL_HEIGHT 100
#define THREAD_DEF_NUM 8
#define POLLING_WAIT_MILLIS 5

typedef int rotation_frame_id_t;
typedef struct {
	rotation_frame_id_t prev_id;
	rotation_frame_id_t curr_id;
	bird_t *bird_ref;
} drawn_bird_t;

typedef struct {
	int first_bird_index, last_bird_index;
	bird_t **birds_copy_to_read;
	bird_t **birds_to_write;
	drawn_bird_t **draw_birds;
	char *output_buf;
	uint8_t **images_data_array;
} btask_args_t;

// The last index needs to be considered exclusive
typedef struct btask_t {
	pthread_mutex_t *buf_mutex;
	void (*task)(pthread_mutex_t *mutex, btask_args_t *args);
	btask_args_t *args;
	struct btask_t *next_task;
} btask_t;

typedef struct {
	pthread_mutex_t *mutex;
	int value;
} atomic_integer_t;

typedef struct {
	btask_t **task_queue;
	pthread_mutex_t *queue_mutex;
	pthread_cond_t *cond;
	atomic_integer_t *atomic_integer;
} thread_args_t;

static const char base64_chars[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int max_payload_len = -1;
static char *control_string = "\033[%d;%dH\033_Ga=T,f=100,q=1,I=%d,X=%d,Y=%d;";
static char *end = "\033\\";
char *base_path = "../resources/bird_";
ssize_t screen_width;
ssize_t screen_heigth;

char new_output_buf[BIRDS_NUM * 100];
int z_index = 0;

void get_image_path(char *base_path, int rotation_frame_id);
int init_rotation_frames(uint8_t **images_data_array, char *base_path);
void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, char *base_path,
		int screen_width, int screen_heigth);
int to_degrees(double radians);
void display_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, char *output_buf);
void update_rotation_frame_id(drawn_bird_t **birds_array);
uint8_t *base64_encode(const uint8_t *input, size_t input_length);
void print_bird(drawn_bird_t **birds_array, uint8_t **images_data_array, int bird_no,
		char *output_buf);
void clean_screen();
void print_unchanged_direction(int id, int x, int y);
void print_changed_direction(int id, int x, int y, uint8_t *payload);
void atomic_increment(atomic_integer_t *atomic_integer);
atomic_integer_t *init_atomic_integer(int value);
bool atomic_is_equal(atomic_integer_t *atomic_integer, int value);
void _task(pthread_mutex_t *buf_mutex, btask_args_t *args);
int get_num_processors();
void print_bird_mutex(drawn_bird_t **birds_array, uint8_t **images_data_array, int bird_no,
		      char *output_buf, pthread_mutex_t *mutex);
void update_rotation_frame_id_parallel(drawn_bird_t **birds_array, int start_index, int last_index);
btask_args_t *btask_args_t_init(int first_index, int last_index, bird_t **birds_copy_to_read,
				bird_t **birds_to_write, drawn_bird_t **draw_birds,
				char *output_buf, uint8_t **images_data_array);
btask_t *btask_t_init(pthread_mutex_t *buf_mutex, btask_args_t *args, btask_t *next_task);
thread_args_t *thread_args_t_init(btask_t **task_queue, pthread_mutex_t *queue_mutex,
				  pthread_cond_t *cond, atomic_integer_t *atomic_integer);
void init_threads(char *output_buf, uint8_t **images_data, drawn_bird_t **draw_birds,
		  bird_t **birds, bird_t **birds_copy, atomic_integer_t **atomic_integer,
		  btask_t **task_queue, btask_t **task_list, pthread_mutex_t **queue_mutex,
		  pthread_cond_t **cond);
void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds, bird_t **birds,
	  bird_t **birds_copy, int *output_buf_len);
void atomic_reset(atomic_integer_t *atomic_integer);
void send_payload_data(uint8_t **payload_data);
int main()
{
	system("clear");
	get_num_processors();
	char *output_buf;
	int output_buf_len;
	uint8_t *images_data[ROTATION_FRAME];
	drawn_bird_t *draw_birds[BIRDS_NUM];
	bird_t *birds[BIRDS_NUM];
	bird_t *birds_copy[BIRDS_NUM];
	atomic_integer_t *atomic_integer;
	btask_t **task_queue_head = (btask_t **)malloc(sizeof(btask_t *));
	btask_t **task_list = (btask_t **)malloc(sizeof(btask_t *));
	pthread_mutex_t *queue_mutex;
	pthread_cond_t *cond;
	init(&output_buf, images_data, draw_birds, birds, birds_copy, &output_buf_len);
	init_threads(output_buf, images_data, draw_birds, birds, birds_copy, &atomic_integer,
		     task_queue_head, task_list, &queue_mutex, &cond);
	send_payload_data(images_data);
	while (1) {
		while (!atomic_is_equal(atomic_integer, THREAD_DEF_NUM)) {
			usleep(1000 * POLLING_WAIT_MILLIS);
		}

		clean_screen();
		printf("\033_Gs=1;\033\\");
		printf("%s", new_output_buf);
		fflush(stdout);
		printf("\033_Gs=2;\033\\");

		new_output_buf[0] = '\0';
		copy(birds, birds_copy, BIRDS_NUM);
		atomic_integer->value = 0;
		*task_queue_head = *task_list;
		pthread_cond_broadcast(cond);
		usleep(1000000 / FRAME_RATE);
	}
}

void get_image_path(char *base_path, int rotation_frame_id)
{
	char buf[20];
	sprintf(buf, "%d", rotation_frame_id);
	strcat(base_path, buf);
	strcat(base_path, ".png");
}

int init_rotation_frames(uint8_t **images_data_array, char *base_path)
{
	for (int i = 0; i < ROTATION_FRAME; i++) {
		char base_path_copy[strlen(base_path) + 20];
		strcpy(base_path_copy, base_path);
		get_image_path(base_path_copy, i);
		FILE *file = fopen(base_path_copy, "rb");
		if (file == NULL) {
			perror("Error during file opening");
			exit(1);
		}
		int size = lseek(fileno(file), 0, SEEK_END);
		lseek(fileno(file), 0, SEEK_SET);
		uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * size);
		if (fread(buf, sizeof(char), size, file) != (unsigned long)size) {
			perror("Error during file reading");
			fclose(file);
			exit(1);
		}
		images_data_array[i] = base64_encode(buf, size);
		fclose(file);
		free(buf);
	}
	return 0;
}

void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, char *base_path,
		int screen_width, int screen_heigth)
{
	for (int i = 0; i < BIRDS_NUM; i++) {
		birds_array[i]->bird_ref =
			init_bird(i, BIRD_WIDTH, BIRD_HEIGTH, screen_width, screen_heigth);
		birds_array[i]->curr_id =
			to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
		birds_array[i]->prev_id = birds_array[i]->curr_id;
	}
	init_rotation_frames(images_data_array, base_path);
}

int to_degrees(double radians)
{
	int deg = (int)(radians * (180.0 / M_PI));
	return (deg % 360 + 360) % 360;
}

void update_rotation_frame_id_parallel(drawn_bird_t **birds_array, int start_index, int last_index)
{
	for (int i = start_index; i < last_index; i++) {
		birds_array[i]->prev_id = birds_array[i]->curr_id;
		birds_array[i]->curr_id =
			to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
	}
}

/**
 * @return an encoded base64 string which is terminated with the null character
 */
uint8_t *base64_encode(const uint8_t *input, size_t input_length)
{

	uint8_t char_array_3[3];
	uint8_t char_array_4[4];
	size_t output_size = ((input_length + 2) / 3) * 4;
	uint8_t *output = (uint8_t *)malloc((output_size + 1) * sizeof(uint8_t));
	int i = 0;
	int chunk_count = 0;

	while (input_length >= 3) {
		input_length -= 3;
		char_array_3[0] = input[i];
		char_array_3[1] = input[i + 1];
		char_array_3[2] = input[i + 2];

		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
		char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
		char_array_4[3] = char_array_3[2] & 0x3f;

		for (size_t j = 0; j < 4; j++) {
			output[chunk_count * 4 + j] = base64_chars[char_array_4[j]];
		}
		i += 3;
		chunk_count++;
	}

	if (input_length > 0) {

		int out_idx = 4 * chunk_count;

		if (input_length == 1) {
			char_array_3[0] = input[i];
			char_array_3[1] = 0;
			char_array_3[2] = 0;
		} else {
			char_array_3[0] = input[i];
			char_array_3[1] = input[i + 1];
			char_array_3[2] = 0;
		}
		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
		char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
		char_array_4[3] = char_array_3[2] & 0x3f;

		output[out_idx++] = base64_chars[char_array_4[0]];
		output[out_idx++] = base64_chars[char_array_4[1]];

		if (input_length == 2) {
			output[out_idx++] = base64_chars[char_array_4[2]];
			output[out_idx++] = '=';
		} else {
			output[out_idx++] = '=';
			output[out_idx++] = '=';
		}
	}
	output[output_size] = '\0';

	if (max_payload_len == -1 || (int)strlen((char *)output) > max_payload_len)
		max_payload_len = (int)strlen((char *)output) + 1;

	return output;
}

void print_bird_mutex(drawn_bird_t **birds_array, uint8_t **images_data_array, int bird_no,
		      char *output_buf, pthread_mutex_t *mutex)
{
	char buf[strlen(control_string) + strlen(end) + 1];
	// char buf1[100];
	buf[strlen(control_string) + strlen(end)] = '\0';
	drawn_bird_t *bird = birds_array[bird_no];
	int col = bird->bird_ref->x / 10;
	int row = bird->bird_ref->y / 20;
	int offset_x = (int)bird->bird_ref->x % 10;
	int offset_y = (int)bird->bird_ref->y % 20;
	rotation_frame_id_t id = bird->curr_id;
	// rotation_frame_id_t prev_id = bird->prev_id;
	// sprintf(buf1, "\033_Ga=d,i=%d,p=%d;\033\\", prev_id, bird_no);
	sprintf(buf, "\033[%d;%dH\033_Ga=p,q=1,I=%d,p=%d,X=%d,Y=%d,z=%d\033\\", row, col, id + 1, 0,
		offset_x, offset_y, z_index++);
	pthread_mutex_lock(mutex);
	// strcat(new_output_buf, buf1);
	strcat(new_output_buf, buf);
	pthread_mutex_unlock(mutex);
}

void send_payload_data(uint8_t **images_data)
{
	for (int i = 0; i < ROTATION_FRAME; i++) {
		printf("\033_Ga=t,f=100,q=2,I=%d;%s\033\\", i + 1, (char *)images_data[i]);
		fflush(stdout);
	}
	clean_screen();
}

// Returns the number of active logic processors of this system
int get_num_processors()

{
	int nprocs = -1;
#ifdef _WIN32
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	nprocs = (long)info.dwNumberOfProcessors;
#else
	nprocs = sysconf(_SC_NPROCESSORS_ONLN);
#endif
	return nprocs;
}

void _task(pthread_mutex_t *buf_mutex, btask_args_t *args)
{
	update_birds_index(args->birds_copy_to_read, args->birds_to_write, screen_width,
			   screen_heigth, args->first_bird_index, args->last_bird_index, BIRDS_NUM);
	update_rotation_frame_id_parallel(args->draw_birds, args->first_bird_index,
					  args->last_bird_index);

	// debug
	if (THREAD_DEF_NUM == 1 &&
	    (!(args->first_bird_index == 0) || !(args->last_bird_index == BIRDS_NUM))) {
		printf("First:%d Last:%d", args->first_bird_index, args->last_bird_index);
		exit(1);
	}
	for (int i = args->first_bird_index; i < args->last_bird_index; i++)
		print_bird_mutex(args->draw_birds, args->images_data_array, i, args->output_buf,
				 buf_mutex);
}

void *thread_main_function(void *args)
{
	thread_args_t *cast = (thread_args_t *)args;
	btask_t **queue = cast->task_queue;
	pthread_mutex_t *mutex = cast->queue_mutex;
	pthread_cond_t *cond = cast->cond;
	atomic_integer_t *atomic_integer = cast->atomic_integer;
	btask_t *task;

	while (1) {
		pthread_mutex_lock(mutex);
		while (*queue == NULL)
			pthread_cond_wait(cond, mutex);
		task = *queue;
		*queue = (*queue)->next_task;
		pthread_mutex_unlock(mutex);

		// Task processing
		pthread_mutex_t *buf_mutex = task->buf_mutex;
		btask_args_t *args = task->args;
		(task->task)(buf_mutex, args);
		atomic_increment(atomic_integer);
	}
	return (void *)(0);
}

void atomic_increment(atomic_integer_t *atomic_integer)
{
	pthread_mutex_lock(atomic_integer->mutex);
	atomic_integer->value = atomic_integer->value + 1;
	pthread_mutex_unlock(atomic_integer->mutex);
}

void atomic_reset(atomic_integer_t *atomic_integer)
{
	pthread_mutex_lock(atomic_integer->mutex);
	atomic_integer->value = 0;
	pthread_mutex_unlock(atomic_integer->mutex);
}

atomic_integer_t *init_atomic_integer(int value)
{
	atomic_integer_t *atomic = (atomic_integer_t *)malloc(sizeof(atomic_integer_t));
	atomic->mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
	pthread_mutex_init(atomic->mutex, NULL);
	atomic->value = value;
	return atomic;
}

bool atomic_is_equal(atomic_integer_t *atomic_integer, int value)
{
	bool cond;
	pthread_mutex_lock(atomic_integer->mutex);
	cond = (atomic_integer->value == value);
	pthread_mutex_unlock(atomic_integer->mutex);
	return cond;
}

btask_args_t *btask_args_t_init(int first_index, int last_index, bird_t **birds_copy_to_read,
				bird_t **birds_to_write, drawn_bird_t **draw_birds,
				char *output_buf, uint8_t **images_data_array)
{
	btask_args_t *args = (btask_args_t *)malloc(sizeof(btask_args_t));
	args->first_bird_index = first_index;
	args->last_bird_index = last_index;
	args->birds_copy_to_read = birds_copy_to_read;
	args->birds_to_write = birds_to_write;
	args->draw_birds = draw_birds;
	args->output_buf = output_buf;
	args->images_data_array = images_data_array;
	return args;
}

btask_t *btask_t_init(pthread_mutex_t *buf_mutex, btask_args_t *args, btask_t *next_task)
{
	btask_t *task = (btask_t *)malloc(sizeof(btask_t));
	task->task = _task;
	task->buf_mutex = buf_mutex;
	task->args = args;
	task->next_task = next_task;
	return task;
}

thread_args_t *thread_args_t_init(btask_t **task_queue, pthread_mutex_t *queue_mutex,
				  pthread_cond_t *cond, atomic_integer_t *atomic_integer)
{

	thread_args_t *args = (thread_args_t *)malloc(sizeof(thread_args_t));
	args->atomic_integer = atomic_integer;
	args->cond = cond;
	args->queue_mutex = queue_mutex;
	args->task_queue = task_queue;
	return args;
}

void init_threads(char *output_buf, uint8_t **images_data, drawn_bird_t **draw_birds,
		  bird_t **birds, bird_t **birds_copy, atomic_integer_t **atomic_integer,
		  btask_t **task_queue_head, btask_t **task_list, pthread_mutex_t **queue_mutex,
		  pthread_cond_t **cond)
{
	btask_t *tail;
	int start_index;
	int end_index;
	int err;
	pthread_t tidp;
	copy(birds, birds_copy, BIRDS_NUM);
	pthread_mutex_t *buf_mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
	*queue_mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
	*cond = (pthread_cond_t *)malloc(sizeof(pthread_cond_t));
	*atomic_integer = init_atomic_integer(0);
	pthread_mutex_init(buf_mutex, NULL);
	pthread_mutex_init(*queue_mutex, NULL);
	pthread_cond_init(*cond, NULL);
	for (int i = 0; i < THREAD_DEF_NUM; i++) {
		start_index = i * (BIRDS_NUM / THREAD_DEF_NUM);
		end_index = (i + 1) * (BIRDS_NUM / THREAD_DEF_NUM);
		if (end_index >= BIRDS_NUM)
			end_index = BIRDS_NUM;

		btask_args_t *task_args =
			btask_args_t_init(start_index, end_index, birds_copy, birds, draw_birds,
					  output_buf, images_data);
		btask_t *task = btask_t_init(buf_mutex, task_args, NULL);
		if (i == 0) {
			*task_queue_head = task;
			*task_list = task;
			tail = task;
		} else {
			tail->next_task = task;
			tail = task;
		}
		thread_args_t *thread_args =
			thread_args_t_init(task_queue_head, *queue_mutex, *cond, *atomic_integer);

		err = pthread_create(&tidp, NULL, thread_main_function, (void *)thread_args);
		if (err != 0)
			fprintf(stderr, "Can't create that thread");
	}
}

void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds, bird_t **birds,
	  bird_t **birds_copy, int *output_buf_len)
{

	struct winsize w;
	ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
	screen_width = w.ws_xpixel;
	screen_heigth = w.ws_ypixel;
	if (!screen_heigth || !screen_width) {
		screen_heigth = DEF_TERMINAL_HEIGHT;
		screen_width = DEF_TERMINAL_WIDTH;
	}
	for (int i = 0; i < BIRDS_NUM; i++) {
		draw_birds[i] = (drawn_bird_t *)malloc(sizeof(drawn_bird_t));
		birds_copy[i] = (bird_t *)malloc(sizeof(bird_t));
	}

	init_birds(draw_birds, images_data, base_path, screen_width, screen_heigth);

	*output_buf_len = strlen(control_string) + max_payload_len + strlen(end);
	*output_buf = (char *)malloc(sizeof(char) * (*output_buf_len) * BIRDS_NUM + 1);
	*output_buf[0] = '\0';

	for (int i = 0; i < BIRDS_NUM; i++) {
		birds[i] = draw_birds[i]->bird_ref;
	}
	new_output_buf[0] = '\0';
}

void clean_screen()
{
	// system("clear");
	printf("\033_Ga=d,d=a\033\\");
}
