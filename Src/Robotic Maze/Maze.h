/*
 * Maze.h
 *
 *  Created on: 24 февр. 2019 г.
 *      Author: wl
 */

#ifndef MAZE_H
#define MAZE_H

#include "configure.h"
#include <stdint.h>

typedef enum {
	straight = 0, 
	right = 1, 
	back = 2, 
	left = 3
} rotation_dir_t;

#define TURN_MASK 0x03

typedef enum {
   north 	= 0,
   east 	= 1,
   south	= 2,
   west		= 3
} bearing_dir_t;

typedef struct coordinate{
	int	east;
	int north;
} coordinate_t; // 8

typedef struct {
	struct coordinate coordinate; // 8
	uint16_t node_link[4];    // 8
	uint8_t pass_count[4];   // 4 
} map_cell_t;  // 20

extern map_cell_t map[MAX_MAP_SIZE];  // 20*256=5120

#define UNKNOWN 0xffff			

extern uint8_t	where_am_i, old_ret;
extern const unsigned int turn_sequence[][3];

unsigned int solveMaze(unsigned int explore_mode);
void BrakeTest(void);
void FreeRun(void);
void Search_Short_Way_with_turns(void);
unsigned int PlayMaze(void);

extern unsigned int calculation_time;

#define ABS(x)  (((x) < 0) ? (-(x)) : (x))

#endif /* MAZE_H_ */
