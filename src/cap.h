#ifndef _CAP_H_
#define _CAP_H_

#include <stdbool.h>
#include <stdint.h>

#define CAP_WIDTH 320
#define CAP_HEIGHT 240
#define CAP_FPS 180

bool cap_init(bool is_replay_, bool is_record_, bool have_meta_);
bool cap_deinit(void);
bool cap_grab(void *raw_image, int *raw_image_size);

#define IMG_H 90 // TODO 根据自己小车的摄像头角度位置填充
#define IMG_W 150 

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_WHITE   "\x1b[37m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#endif