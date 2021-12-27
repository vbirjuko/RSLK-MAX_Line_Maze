/*
 * square_maze.h
 *
 *  Created on: 5 янв. 2021 г.
 *      Author: wl
 */

#ifndef SQUARE_MAZE_H_
#define SQUARE_MAZE_H_

#define MAZE_FACTOR     4
#define MAZE_SIDE   (1 << MAZE_FACTOR)

typedef struct {
    unsigned int path_size;
    int * pathlength;
    uint8_t * path;
    int * length;
} path_t;

void init_maze(unsigned int x_size, unsigned int y_size, coordinate_t start_cell, bearing_dir_t start_dir);
void draw_maze (coordinate_t cursor, path_t * show_path);
void solve_sq_maze(void);

#endif /* SQUARE_MAZE_H_ */
