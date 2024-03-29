/*
 * square_maze.c
 *
 *  Created on: 5 янв. 2021 г.
 *      Author: wl
 */

#include "display.h"
#include "dma.h"
#include "CortexM.h"
#include "Maze.h"
#include "color_sensor.h"
#include "Timer32.h"
#include "rand.h"
#include "display.h"
#include "configure.h"
#include "square_maze.h"

// for debug purposes
#include "LaunchPad.h"

unsigned int search_way_simple(coordinate_t start, coordinate_t finish,
                               bearing_dir_t prev_bearing, unsigned int open);
unsigned int test_route(coordinate_t * my_place, bearing_dir_t my_direction);

#define OLD_VARIANT
#define SORTED_QUEUE

#define QUEUE_FACTOR  9
#define QUEUE_SIZE  (1 << QUEUE_FACTOR)
#define QUEUE_MASK  (QUEUE_SIZE - 1)

static unsigned int flood[MAZE_SIDE*MAZE_SIDE*2];

#define USE_DMA

#define MAX_PATH_LENGTH (MAX_MAP_SIZE * 2)

path_t global_path = {
       MAX_MAP_SIZE * 2,
       &data.pathlength,
       data.path,
       data.length
};

int backpath_size;
uint8_t backpath_turn[256];
int backpath_length[256];

path_t back_path = {
        256,
        &backpath_size,
        backpath_turn,
        backpath_length
};


enum sides {
    north_wall = 0x01,
    east_wall  = 0x02,
    south_wall = 0x04,
    west_wall  = 0x08
};

uint8_t maze[MAZE_SIDE * MAZE_SIDE], maxX, maxY;
coordinate_t start, finish={16, 16};
bearing_dir_t start_direction=north;

coordinate_t my_coordinate;
uint8_t maze_count[256][4] = {{0, 0, 0, 0}};
bearing_dir_t my_bearing;

unsigned int flood_array[MAZE_SIDE * MAZE_SIDE * 2];

unsigned int update_cell_walls(coordinate_t start_cell, bearing_dir_t start_direction, unsigned int start_cell_walls) {
    unsigned int cell_walls, prev_cell_walls;

    prev_cell_walls = maze[start_cell.north * MAZE_SIDE + start_cell.east];
    cell_walls = start_cell_walls << start_direction;
    cell_walls |= (cell_walls << 4) | (cell_walls >> 4);

    maze[start_cell.north * MAZE_SIDE + start_cell.east] = cell_walls;

    if (start_cell.east < maxX)
        if (cell_walls & (east_wall | (east_wall << 4)))
            maze[start_cell.north * MAZE_SIDE + start_cell.east + 1] |= (west_wall | (west_wall <<4));
        else
            maze[start_cell.north * MAZE_SIDE + start_cell.east + 1] &= ~(west_wall | (west_wall <<4));
    if (start_cell.east > 0)
        if (cell_walls & (west_wall | (west_wall << 4)))
            maze[start_cell.north * MAZE_SIDE + start_cell.east - 1] |= (east_wall | (east_wall <<4));
        else
            maze[start_cell.north * MAZE_SIDE + start_cell.east - 1] &= ~(east_wall | (east_wall <<4));
    if (start_cell.north < maxY)
        if (cell_walls & (north_wall | (north_wall << 4)))
            maze[(start_cell.north+1) * MAZE_SIDE + start_cell.east] |= (south_wall | (south_wall <<4));
        else
            maze[(start_cell.north+1) * MAZE_SIDE + start_cell.east] &= ~(south_wall | (south_wall <<4));
    if (start_cell.north > 0)
        if (cell_walls & (south_wall | (south_wall << 4)))
            maze[(start_cell.north-1) * MAZE_SIDE + start_cell.east] |= (north_wall | (north_wall <<4));
        else
            maze[(start_cell.north-1) * MAZE_SIDE + start_cell.east] &= ~(north_wall | (north_wall <<4));

    return maze[start_cell.north * MAZE_SIDE + start_cell.east] != prev_cell_walls;
}

void init_maze(unsigned int x_size, unsigned int y_size, coordinate_t start_cell, bearing_dir_t start_dir) {
    unsigned int ii;

    if (x_size * y_size > 256) return;
    if (start_cell.north >= y_size) return;
    if (start_cell.east  >= x_size)  return;

#ifdef USE_DMA
    fill_data8_dma(0x0f, maze, 256); //  все стены неопределены.
#endif
    maxX = x_size;
    maxY = y_size;
    start_direction = start_dir;
    start.east =  (data.sq_init & 0x00F000) >> 12;
    start.north = (data.sq_init & 0x000F00) >>  8;


#ifndef USE_DMA
    for (ii = 0; ii < MAZE_SIDE*MAZE_SIDE; ii++){
        maze[ii] = 0x0f; //  все стены неопределены.
    }
    for (ii = 0; ii < MAZE_SIDE; ii++){
        maze[ii] |= south_wall | (south_wall << 4); //  определем южную стену.
        maze[ii + (MAZE_SIDE - 1) * MAZE_SIDE] |= north_wall | (north_wall << 4); // северную стену
    }
#else
    while(dma_copy_busy) WaitForInterrupt();
    fill_data8_dma((south_wall << 4) | 0x0f, &maze[0], MAZE_SIDE); //  определем южную стену.
    while(dma_copy_busy) WaitForInterrupt();
    fill_data8_dma((north_wall << 4) | 0x0f, &maze[(maxY-1)*MAZE_SIDE], MAZE_SIDE); // северную стену.
    while(dma_copy_busy) WaitForInterrupt();
    fill_data8_dma((north_wall << 4) | 0x0f, &maze[(MAZE_SIDE-1)*MAZE_SIDE], MAZE_SIDE); // "дальнюю" северную стену.
    while(dma_copy_busy) WaitForInterrupt();
#endif



    for (ii = 0; ii < MAZE_SIDE; ii++){
        maze[ii * MAZE_SIDE] |= west_wall | (west_wall << 4); // западную стену
        maze[ii * MAZE_SIDE + maxX - 1] |= east_wall | (east_wall << 4); //  восточную стену.
        maze[(ii+1) * MAZE_SIDE - 1] |= east_wall | (east_wall << 4); //  и "дальнюю" восточную стену.
    }
    update_cell_walls(start_cell, start_direction, (east_wall | south_wall | west_wall));
}

static unsigned int update_coordinate(coordinate_t * current_coordinate, bearing_dir_t direction) {
    switch (direction) {
    case north:
        current_coordinate->north++;
        if (current_coordinate->north >= MAZE_SIDE)return 1;
        break;
    case east:
        current_coordinate->east++;
        if (current_coordinate->east >= MAZE_SIDE) return 1;
        break;
    case south:
        if (current_coordinate->north) current_coordinate->north--;
        else return 1;
        break;
    case west:
        if (current_coordinate->east) current_coordinate->east--;
        else return 1;
        break;
    }
    return 0;
}

void draw_maze (coordinate_t cursor, path_t * show_path) {
    unsigned int p_step, ii, jj, linetype;
    const unsigned int table[] = {0, 2, 3, 1};

    p_step = 127/maxX;
    if (p_step > 64/maxY) p_step = 64/maxY;

    squareXY(0,0, 127, 63, 0);
    for (ii = 0; ii < maxY; ii++) {
        for (jj = 0; jj < maxX; jj++) { // западная стена
            linetype = 0;
            if (maze[ii*MAZE_SIDE + jj] & west_wall) linetype |= 1;
            if (maze[ii*MAZE_SIDE + jj] & (west_wall << 4)) linetype |= 2;
            linetype = table[linetype];
            squareXY(jj * p_step, 63-  (ii   * p_step),
                     jj * p_step, (((ii+1)* p_step) > 63) ? 0 : (63- ((ii+1)* p_step)),
                     linetype);
            if ( jj == (maxX - 1)) { // восточная стена рисуется только у правого столбца
                linetype = 0;
                if (maze[ii*MAZE_SIDE + jj] &  east_wall) linetype |= 1;
                if (maze[ii*MAZE_SIDE + jj] & (east_wall << 4)) linetype |= 2;
                linetype = table[linetype];
                squareXY(  (jj+1)*p_step, 63- (ii   * p_step),
                         (jj+1)*p_step, (((ii+1)* p_step) > 63) ? 0 : (63- ((ii+1)* p_step)),
                         linetype);
            }
            // южная стена
            linetype = 0;
            if (maze[ii*MAZE_SIDE + jj] & south_wall) linetype |= 1;
            if (maze[ii*MAZE_SIDE + jj] & (south_wall << 4)) linetype |= 2;
            linetype = table[linetype];
            squareXY(jj   * p_step,   63-(ii*p_step),
                    (jj+1)* p_step,   63-(ii*p_step),
                    linetype);
            if (ii == (maxY - 1)) {  // северная стена тоже рисуется только у верхнего ряда
                linetype = 0;
                if (maze[ii*MAZE_SIDE + jj] & north_wall) linetype |= 1;
                if (maze[ii*MAZE_SIDE + jj] & (north_wall << 4)) linetype |= 2;
                linetype = table[linetype];
                squareXY(jj   * p_step,  63-((ii+1) * p_step),
                        (jj+1)* p_step,  63-((ii+1) * p_step),
                        linetype);
            }
        }
    }
    squareXY((start.east*p_step+2), 63- (start.north*p_step+2),
            ((start.east+1)*p_step-2), 63-((start.north+1)*p_step-2), 1);

    squareXY((start.east*p_step+3), 63- (start.north*p_step+3),
            ((start.east+1)*p_step-3), 63-((start.north+1)*p_step-3), 0);

    squareXY((finish.east*p_step+2), 63- (finish.north*p_step+2),
            ((finish.east+1)*p_step-2), 63-((finish.north+1)*p_step-2), 1);

    squareXY((cursor.east*p_step+3), 63- (cursor.north*p_step+3),
            ((cursor.east+1)*p_step-3), 63-((cursor.north+1)*p_step-3), 1);

    // Теперь маршрут
    if (*(show_path->pathlength))    {
        unsigned int direction;
        coordinate_t prev_pos, curr_pos;
        int path_step = 0;

        curr_pos = prev_pos = start; direction = start_direction;

        while (path_step < *(show_path->pathlength)) {
            if (update_coordinate(&curr_pos, (bearing_dir_t)direction)) break;

            lineXY(prev_pos.east*p_step + p_step/2, 63 - (prev_pos.north*p_step + p_step/2),
                   curr_pos.east*p_step + p_step/2, 63 - (curr_pos.north*p_step + p_step/2), 1);

            prev_pos = curr_pos;
// Если попали в "туннель", то продолжаем движение в том же направлении.
            if (maze[curr_pos.north*MAZE_SIDE+curr_pos.east] == 0x55) continue;
            if (maze[curr_pos.north*MAZE_SIDE+curr_pos.east] == 0xaa) continue;

            direction +=  show_path->path[path_step++]; // data.path[path_step++];
            direction &= TURN_MASK;
        }
    }
    update_display();
}

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variable available_directions which indicate
// whether there is an exit in each of the three directions. If no - turn back
static rotation_dir_t SelectTurn(unsigned int unavailable_directions) {
    const unsigned int try_sequence[][3] = {
                                            {east_wall, north_wall, west_wall},
                                            {west_wall, north_wall, east_wall},
                                            {east_wall, west_wall, north_wall},
                                            {west_wall, east_wall, north_wall},
                                            {north_wall, east_wall, west_wall},
                                            {north_wall, west_wall, east_wall},
    };

    unsigned int  i, turn_index, result[3], min_result = 0xffffFFFF, min_index = 0;
    coordinate_t next_possible;
    unsigned int try_bearing;

    turn_index = ((unsigned int)data.lefthand < 6u) ? data.lefthand : Rand() % 6;

    if (unavailable_directions != (west_wall | north_wall | east_wall)) {
        for (i = 0; i < 4; i++) {
            if (~unavailable_directions & try_sequence[turn_index][i]) {
                //                break;
                next_possible = my_coordinate;
                switch (try_sequence[turn_index][i]) {
                    case north_wall:
                        try_bearing = my_bearing;
                        break;
                    case west_wall:
                        try_bearing = (bearing_dir_t)((my_bearing + left) & TURN_MASK);
                        break;
                    case east_wall:
                        try_bearing = (bearing_dir_t)((my_bearing + right) & TURN_MASK);
                        break;
                    case south_wall:
                        try_bearing = (bearing_dir_t)((my_bearing + back) & TURN_MASK);
                }
                update_coordinate(&next_possible, (bearing_dir_t) try_bearing);
#ifdef OLD_VARIANT
                result[i] = ((next_possible.north > 7) ? (next_possible.north - 8)*(next_possible.north - 8) : (7 - next_possible.north)*(7 - next_possible.north)) +
                ((next_possible.east  > 7) ? (next_possible.east  - 8)*(next_possible.east  - 8) : (7 - next_possible.east)*(7 - next_possible.east));
#else
                result[i] = flood[(next_possible.north*maxY + next_possible.east)*2 + (try_bearing & 0x01)];
#endif
            } else {
                result[i] = 0xffffFFFF;
            }
        }
        for (i = 0; i < 3; i++) {
            if (min_result > result[i]) {
                min_index = i;
                min_result = result[i];
            }
        }
        switch (try_sequence[turn_index][min_index]) {
            case north_wall:
                return straight;
            case west_wall:
                return left;
            case east_wall:
                return right;
            default:
                return back;
        }
    }
    return back;
}

static void simplifyPath(void) {
    int total_angle = straight;
    int i;

    // only simplify the path if the second-to-last turn was a "Back"
    if(data.pathlength < 3 || data.path[data.pathlength - 2] != back) return;


    for(i = 1; i <= 3; i++) total_angle += data.path[data.pathlength - i];
    total_angle &= TURN_MASK;
    data.path[data.pathlength - 3] = (rotation_dir_t)total_angle;
    // The path is now two steps shorter.
    data.pathlength -= 2;
}

unsigned int do_prediction(coordinate_t from_coordinate, bearing_dir_t possible_bearing) {
    unsigned int count_green = 0, count_yellow = 0, count_red = 0, possible_walls, ii, try_bearing;
    coordinate_t possible_destination;
    uint8_t counts[4];

    // проверяем на возможно тупиковый путь
    possible_destination = from_coordinate;
    try_bearing = possible_bearing;
    do {
        update_coordinate(&possible_destination, (bearing_dir_t)try_bearing);
        possible_walls = maze[possible_destination.north*MAZE_SIDE + possible_destination.east];
        possible_walls &= 0xF0;
        possible_walls >>= try_bearing;
        possible_walls = (possible_walls | (possible_walls >> 4)) & 0x0F;
        if ((possible_walls & (west_wall | east_wall | north_wall)) == (west_wall | east_wall | north_wall)) {
            maze_count[from_coordinate.north*MAZE_SIDE + from_coordinate.east][possible_bearing] = 2;
            return 1;
        }
        if ((possible_walls & (west_wall | east_wall)) == (west_wall | east_wall)) continue;
        if ((possible_walls & (north_wall | west_wall)) == (north_wall | west_wall)) {
            try_bearing = (try_bearing + right) & TURN_MASK;
            continue;
        }
        if ((possible_walls & (north_wall | east_wall)) == (north_wall | east_wall)) {
            try_bearing = (try_bearing + left) & TURN_MASK;
            continue;
        }
        break;
    } while(1);

    possible_destination = from_coordinate;
    try_bearing = possible_bearing;
    do {
        update_coordinate(&possible_destination, (bearing_dir_t)try_bearing);
        possible_walls = maze[possible_destination.north*MAZE_SIDE + possible_destination.east];
        if ((possible_walls & 0x0f) != ((possible_walls & 0xf0)>>4)) return 0;   // мы в этой ячейке еще точно не были - надо посетить
        if ((possible_walls == 0xaa) || (possible_walls == 0x55)) continue; // если попали в "туннель" повторяем движение.
                                          // 0000 0000 aaaa bbbb
        possible_walls >>= try_bearing;     // 0000 0000 000a aaab
        possible_walls &= 0x0f;

        if ((possible_walls & (west_wall | east_wall)) == (west_wall | east_wall)) continue;
        if ((possible_walls & (north_wall | west_wall)) == (north_wall | west_wall)) {
            try_bearing = (try_bearing + right) & TURN_MASK;
            continue;
        }
        if ((possible_walls & (north_wall | east_wall)) == (north_wall | east_wall)) {
            try_bearing = (try_bearing + left) & TURN_MASK;
            continue;
        }

        break;
    } while(1);

    for (ii = 0; ii < 4; ii++) {
        counts[ii] = maze_count[possible_destination.north*MAZE_SIDE + possible_destination.east][ii];
    }
    counts[(try_bearing + back) & TURN_MASK] += 1;
//    unavailable = 0; // допустим стен нет
    for (ii = 0; ii < 4; ii++) {
//        if (unavailable & (1 << ii)) continue; // если есть стена, то эту сторону пропускаем
        switch (counts[(possible_bearing+ii) & TURN_MASK]) {
            case 0: count_green++;  break;
            case 1: count_yellow++; break;
            default: count_red++;    break;
        }
    }
    if (count_yellow != 3) return 0;
// есть одиночная нить - мы там были и туда суваться нет надобности - отмечаем, что мы там были и вернулись
    maze_count[possible_destination.north*MAZE_SIDE + possible_destination.east][(try_bearing + back) & TURN_MASK] += 2;
    maze_count[from_coordinate.north*MAZE_SIDE + from_coordinate.east][possible_bearing] += 2;

    return 1;
}

#define swap(a, b) \
(((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

void do_transpose(unsigned int count) {
    unsigned int aa, bb, cc, dd, ii;
    aa = (maze[0] & 0xAA) >> 1;
    bb = (maze[0] & 0x55) << 1;
    maze[0] = aa | bb;
    swap(maze_count[0][0],maze_count[0][1]);
    swap(maze_count[0][2],maze_count[0][3]);
    for (ii = 1; ii <= count; ii++) {
        aa = (maze[ii] & 0xAA) >> 1;
        bb = (maze[ii] & 0x55) << 1;
        cc = (maze[MAZE_SIDE*ii] & 0xAA) >> 1;
        dd = (maze[MAZE_SIDE*ii] & 0x55) << 1;
        maze[MAZE_SIDE*ii] = aa | bb;
        maze[ii] = cc | dd;
        swap(maze_count[ii][0], maze_count[MAZE_SIDE*ii][1]);
        swap(maze_count[ii][1], maze_count[MAZE_SIDE*ii][0]);
        swap(maze_count[ii][2], maze_count[MAZE_SIDE*ii][3]);
        swap(maze_count[ii][3], maze_count[MAZE_SIDE*ii][2]);
    }
}

__WEAK uint8_t get_walls(void) {
    return north_wall | east_wall | west_wall;
}

__WEAK unsigned int make_step(void) {
    return 3;
}

__WEAK void init_virtual_maze(void) {
    return;
}

__WEAK bearing_dir_t make_turn(rotation_dir_t turn_direction) {
    return north;
}

__WEAK color_t get_color(void) {
    return white;
}

#ifdef OLD_VARIANT
void solve_sq_maze(void) {
    rotation_dir_t turn_direction;
    unsigned int current_cell_walls = 0;
    unsigned int steps, count_green, count_yellow, count_red, ii, skiplevel, unavailable;
    //    unsigned int open_cost, close_cost;
    unsigned int my_next_possible_bearing, back_bearing, finish_found = 0;
    //    coordinate_t check_coordinate;
    start.east =  (data.sq_init & 0x00F000) >> 12;
    start.north = (data.sq_init & 0x000F00) >>  8;
    my_coordinate = start;
    //    my_coordinate.east = (data.sq_init & 0x0F000) >> 12;
    //    my_coordinate.north= (data.sq_init & 0x00F00) >> 8;
    back_bearing = my_bearing = (bearing_dir_t)((data.sq_init & 0xF0000) >> 16);
    fill_data32_dma(0, (uint32_t *)maze_count, 256);
    unsigned int first_turn = 1;

    draw_maze(my_coordinate, &global_path);
    init_virtual_maze();
    data.pathlength = 0;
    while (dma_copy_busy) continue;

    do {
        unsigned int recalculate = 0;
        maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][my_bearing] += 1;
        steps = make_step();
        data.length[data.pathlength] = steps * data.cell_step;  // до поворота
        while (steps-- > 1) {
            current_cell_walls = (east_wall | west_wall);
            update_coordinate(&my_coordinate, my_bearing);
            maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][my_bearing] += 1;
            maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+back) & TURN_MASK] += 1;
            update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
            draw_maze(my_coordinate, &global_path);
            delay_us(1000000);
        }
        update_coordinate(&my_coordinate, my_bearing);
        maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+back) & TURN_MASK] += 1;
        current_cell_walls = get_walls();
        recalculate |= update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
        if (first_turn && ((current_cell_walls & (west_wall | east_wall) )!= (west_wall | east_wall))) {
            if ((current_cell_walls & west_wall) == 0) {
                do_transpose(my_coordinate.north);
                my_bearing = east;
            }
            first_turn = 0;
        }
        draw_maze(my_coordinate, &global_path);

        // варианты нахождения финиша:
        if (current_cell_walls == (west_wall | north_wall | east_wall)) {
            if (get_color() == red) {
                finish = my_coordinate;
                finish_found = 1;
                //                break;
            }
        }
        // Финиш в центре лабиринта 16х16
        if ((my_coordinate.east == 7 || my_coordinate.east == 8)&& (my_coordinate.north == 7 || my_coordinate.north == 8)) {
            if (!finish_found) finish = my_coordinate;
            finish_found = 1;
            //            break;
        }
        delay_us(1000000);

        if (finish_found  & recalculate) {
            recalculate = 0;
            unsigned int opencost = search_way_simple(start, finish, (bearing_dir_t)((data.sq_init & 0xF0000) >> 16), 1);
            if (opencost >= search_way_simple(start, finish, (bearing_dir_t)((data.sq_init & 0xF0000) >> 16), 0)) {
                LaunchPad_Output(GREEN | BLUE);
                break;
            }
        }
        do {
            //            luc_tremo:
            if (((current_cell_walls & west_wall) == 0) &&
                (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][((my_bearing + left) & TURN_MASK)] == 0))
                    do_prediction(my_coordinate, (bearing_dir_t)((my_bearing + left) & TURN_MASK));
            if (((current_cell_walls & east_wall) == 0) &&
                (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][((my_bearing + right) & TURN_MASK)] == 0))
                    do_prediction(my_coordinate, (bearing_dir_t)((my_bearing + right) & TURN_MASK));
            if (((current_cell_walls & north_wall) == 0) &&
                (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][my_bearing] == 0))
                    do_prediction(my_coordinate, my_bearing);

            count_green = 0; count_yellow = 0; count_red = 0;
            unavailable = current_cell_walls;
            for (ii = 0; ii < 4; ii++) {
                if (unavailable & (1 << ii)) {
                    maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+ii) & TURN_MASK] = 3;
                    continue; // если есть стена, то эту сторону пропускаем
                }
                switch (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+ii) & TURN_MASK]) {
                case 0: count_green++;  break;
                case 1: count_yellow++; break;
                default: count_red++;    break;
                }
            }
            if (count_yellow == 3) skiplevel = 0; // есть одиночная нить пересекающая узел - возвращаемся
            else if (count_green)  skiplevel = 1;  // оставить только зелёные проходы
            else if (count_yellow) skiplevel = 2;  // оставить все не красные проходы
            else goto save_map;   // если нет ни зелёного, ни желтого прохода - больше идти некуда.

            if (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+left) & TURN_MASK] >= skiplevel) unavailable |= west_wall;
            if (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+right) & TURN_MASK] >= skiplevel) unavailable |= east_wall;
            if (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+straight) & TURN_MASK] >= skiplevel) unavailable |= north_wall;

            turn_direction = SelectTurn(unavailable);
            my_next_possible_bearing = my_bearing;
            my_next_possible_bearing += turn_direction;
            my_next_possible_bearing &= TURN_MASK;

            //            if ((turn_direction == back)) {
            //                if ((data.pathlength > 2) &&
            //                        (data.path[data.pathlength-1] == data.path[data.pathlength-2]) &&
            //                        ((data.path[data.pathlength-1] == left) || (data.path[data.pathlength-1] == right))) {
            //                    if (!finish_found) finish = my_coordinate;
            //                }
            //            }


            if (skiplevel != 1) {
                // если возвращаемся назад, то давайте посмотрим, как далеко нужно идти назад.
                // для этого возвращаемся по жёлтому пути и ищем ячейку у которой есть
                // зелёный проход (count == 0). После чего расчитываем до той ячейки кратчайший маршрут
                // и перемещаемся туда.

                coordinate_t back_trace = my_coordinate;

                back_bearing = my_bearing;

                do {
                    back_bearing += turn_direction;
                    back_bearing &= TURN_MASK;
                    data.path[data.pathlength++] = turn_direction;
                    simplifyPath();
                    do {
                        maze_count[back_trace.north*MAZE_SIDE + back_trace.east][back_bearing] += 1;
                        update_coordinate(&back_trace, (bearing_dir_t)back_bearing);
                        maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+back) & TURN_MASK] += 1;
                        current_cell_walls = maze[back_trace.north * MAZE_SIDE + back_trace.east];
                        current_cell_walls >>= back_bearing;     // 0000 0000 000a aaab
                        current_cell_walls &= 0x0f;
                    } while ((current_cell_walls & (west_wall | east_wall)) == (west_wall | east_wall));
                    if (((current_cell_walls & west_wall) == 0) && (
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][((back_bearing + left) & TURN_MASK)] == 0))
                        do_prediction(back_trace, (bearing_dir_t)((back_bearing + left) & TURN_MASK));
                    if (((current_cell_walls & east_wall) == 0) && (
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][((back_bearing + right) & TURN_MASK)] == 0))
                        do_prediction(back_trace, (bearing_dir_t)((back_bearing + right) & TURN_MASK));
                    if (((current_cell_walls & north_wall) == 0) && (
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][back_bearing] == 0))
                        do_prediction(back_trace, (bearing_dir_t) back_bearing);

                    count_green = 0; count_yellow = 0; count_red = 0;
                    unavailable = current_cell_walls;
                    for (ii = 0; ii < 4; ii++) {
                        if (unavailable & (1 << ii)) {
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+ii) & TURN_MASK] = 3;
                            continue; // если есть стена, то эту сторону пропускаем
                        }
                        switch (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+ii) & TURN_MASK]) {
                        case 0: count_green++;  break;
                        case 1: count_yellow++; break;
                        default: count_red++;    break;
                        }
                    }
                    if (count_yellow == 3) skiplevel = 0; // есть одиночная нить пересекающая узел - возвращаемся
                    else if (count_green)  break;  // есть зелёные проходы - конец отката.
                    else if (count_yellow) skiplevel = 2;  // оставить все не красные проходы
                    else break; //goto save_map;   // если нет ни зелёного, ни желтого прохода - больше идти некуда.

                    if (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+left) & TURN_MASK] >= skiplevel) unavailable |= west_wall;
                    if (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+right) & TURN_MASK] >= skiplevel) unavailable |= east_wall;
                    if (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+straight) & TURN_MASK] >= skiplevel) unavailable |= north_wall;
                    turn_direction = SelectTurn(unavailable);
                } while(1);
                // Телепортируемся
                draw_maze(my_coordinate, &global_path);
                LaunchPad_Output(BLUE);
                search_way_simple(my_coordinate, back_trace, my_bearing, 0);
                for (ii = 0; ii < *back_path.pathlength; ii++) {
                    make_turn((rotation_dir_t)back_path.path[ii]);
                    my_bearing += back_path.path[ii];
                    my_bearing &= TURN_MASK;
                    steps = make_step();
                    while (steps-- > 1) {
                        update_coordinate(&my_coordinate, my_bearing);
                        draw_maze(my_coordinate, &global_path);
                        delay_us(1000000);
                    }
                    update_coordinate(&my_coordinate, my_bearing);
                    current_cell_walls = get_walls();
                    update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
                    draw_maze(my_coordinate, &global_path);
                    //                    if (current_cell_walls == (west_wall | north_wall | east_wall)) {
                    //                        if (get_color() == red) break;
                    //                    }
                    delay_us(1000000);
                }
                //                my_bearing = make_turn((rotation_dir_t)((back_bearing - my_bearing) & TURN_MASK));
                current_cell_walls = get_walls();
            } else {
                LaunchPad_Output(0);
                break;
            }
        } while (1);


        make_turn(turn_direction);
        my_bearing += turn_direction;
        my_bearing &= TURN_MASK;
        //        data.path[data.pathlength] = turn_direction;
        data.path[data.pathlength] = (my_bearing - back_bearing) & TURN_MASK;
        data.pathlength++;
        back_bearing = my_bearing;
        // You should check to make sure that the path_length does not
        // exceed the bounds of the array.
        if (data.pathlength >= MAX_PATH_LENGTH) {
            squareXY(0,0, 127, 63, 1);
            update_display();
            putstr(0, 3, "MAX PATH LENGTH", 1);
            //            Motor_Speed(0, 0);
            //            speed  = 0;
            //            where_am_i = old_ret;
            return ;//1;
        }

        // Simplify the learned path.
        simplifyPath();


    } while (1);




save_map:
    search_way_simple(my_coordinate, start, my_bearing, 0);
    for (ii = 0; ii < *back_path.pathlength; ii++) {
        make_turn((rotation_dir_t)back_path.path[ii]);
        my_bearing +=  back_path.path[ii];
        my_bearing &= TURN_MASK;
        steps = make_step();
        while (steps--) {
            update_coordinate(&my_coordinate, my_bearing);
            draw_maze(my_coordinate, &global_path);
            if (steps) {
                current_cell_walls = (east_wall | west_wall);
                update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
                delay_us(1000000);
            }
        }
        draw_maze(my_coordinate, &global_path);
        delay_us(1000000);
    }

    search_way_simple(start, finish, (bearing_dir_t)((data.sq_init & 0xF0000) >> 16), 0);
    copy_data_dma(backpath_turn+1, data.path, backpath_size-1);
    data.pathlength = backpath_size-1;
    while(dma_copy_busy) WaitForInterrupt();
    draw_maze(my_coordinate, &global_path);
    LaunchPad_Output(GREEN);
    return;
}
#else
void solve_sq_maze(void) {
    rotation_dir_t turn_direction;
    unsigned int current_cell_walls = 0;
    unsigned int steps, count_green, count_yellow, count_red, ii, skiplevel, unavailable;
    //    unsigned int open_cost, close_cost;
    unsigned int my_next_possible_bearing, back_bearing, finish_found = 0;
    //    coordinate_t check_coordinate;
    start.east =  (data.sq_init & 0x00F000) >> 12;
    start.north = (data.sq_init & 0x000F00) >>  8;
    my_coordinate = start;
    //    my_coordinate.east = (data.sq_init & 0x0F000) >> 12;
    //    my_coordinate.north= (data.sq_init & 0x00F00) >> 8;
    back_bearing = my_bearing = (bearing_dir_t)((data.sq_init & 0xF0000) >> 16);
    fill_data32_dma(0, (uint32_t *)maze_count, 256);
    unsigned int first_turn = 1;

    draw_maze(my_coordinate, &global_path);
    init_virtual_maze();
    data.pathlength = 0;
    while (dma_copy_busy) continue;

    do {
        unsigned int recalculate = 0;
        maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][my_bearing] += 1;
        steps = make_step();
        data.length[data.pathlength] = steps * data.cell_step;  // до поворота
        while (steps-- > 1) {
            current_cell_walls = (east_wall | west_wall);
            update_coordinate(&my_coordinate, my_bearing);
            maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][my_bearing] += 1;
            maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+back) & TURN_MASK] += 1;
            update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
            draw_maze(my_coordinate, &global_path);
            delay_us(1000000);
        }
        update_coordinate(&my_coordinate, my_bearing);
        maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+back) & TURN_MASK] += 1;
        current_cell_walls = get_walls();
        recalculate |= update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
        if (first_turn && ((current_cell_walls & (west_wall | east_wall) )!= (west_wall | east_wall))) {
            if ((current_cell_walls & west_wall) == 0) {
                do_transpose(my_coordinate.north);
                my_bearing = east;
            }
            first_turn = 0;
        }
        draw_maze(my_coordinate, &global_path);

        // варианты нахождения финиша:
        if (current_cell_walls == (west_wall | north_wall | east_wall)) {
            if (get_color() == red) {
                finish = my_coordinate;
                finish_found = 1;
                //                break;
            }
        }
        // Финиш в центре лабиринта 16х16
        if ((my_coordinate.east == 7 || my_coordinate.east == 8)&& (my_coordinate.north == 7 || my_coordinate.north == 8)) {
            if (!finish_found) finish = my_coordinate;
            finish_found = 1;
            //            break;
        }
        delay_us(1000000);

        if (finish_found  & recalculate) {
            recalculate = 0;
            unsigned int opencost =search_way_simple(start, finish, (bearing_dir_t)((data.sq_init & 0xF0000) >> 16), 1);
            if (opencost >= search_way_simple(start, finish, (bearing_dir_t)((data.sq_init & 0xF0000) >> 16), 0)) {
                LaunchPad_Output(GREEN | BLUE);
                break;
            }
        }
        do {
            //            luc_tremo:
            if (((current_cell_walls & west_wall) == 0) && (
                    maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][((my_bearing + left) & TURN_MASK)] == 0))
                do_prediction(my_coordinate, (bearing_dir_t)((my_bearing + left) & TURN_MASK));
            if (((current_cell_walls & east_wall) == 0) && (
                    maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][((my_bearing + right) & TURN_MASK)] == 0))
                do_prediction(my_coordinate, (bearing_dir_t)((my_bearing + right) & TURN_MASK));
            if (((current_cell_walls & north_wall) == 0) && (
                    maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][my_bearing] == 0))
                do_prediction(my_coordinate, my_bearing);

            count_green = 0; count_yellow = 0; count_red = 0;
            unavailable = current_cell_walls;
            for (ii = 0; ii < 4; ii++) {
                if (unavailable & (1 << ii)) {
                    maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+ii) & TURN_MASK] = 3;
                    continue; // если есть стена, то эту сторону пропускаем
                }
                switch (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+ii) & TURN_MASK]) {
                case 0: count_green++;  break;
                case 1: count_yellow++; break;
                default: count_red++;    break;
                }
            }
            if (count_yellow == 3) skiplevel = 0; // есть одиночная нить пересекающая узел - возвращаемся
            else if (count_green)  skiplevel = 1;  // оставить только зелёные проходы
            else if (count_yellow) skiplevel = 2;  // оставить все не красные проходы
            else goto save_map;   // если нет ни зелёного, ни желтого прохода - больше идти некуда.

            if (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+left) & TURN_MASK] >= skiplevel) unavailable |= west_wall;
            if (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+right) & TURN_MASK] >= skiplevel) unavailable |= east_wall;
            if (maze_count[my_coordinate.north*MAZE_SIDE + my_coordinate.east][(my_bearing+straight) & TURN_MASK] >= skiplevel) unavailable |= north_wall;

            turn_direction = SelectTurn(unavailable);
            my_next_possible_bearing = my_bearing;
            my_next_possible_bearing += turn_direction;
            my_next_possible_bearing &= TURN_MASK;

            //            if ((turn_direction == back)) {
            //                if ((data.pathlength > 2) &&
            //                        (data.path[data.pathlength-1] == data.path[data.pathlength-2]) &&
            //                        ((data.path[data.pathlength-1] == left) || (data.path[data.pathlength-1] == right))) {
            //                    if (!finish_found) finish = my_coordinate;
            //                }
            //            }


            if (skiplevel != 1) {
                // если возвращаемся назад, то давайте посмотрим, как далеко нужно идти назад.
                // для этого возвращаемся по жёлтому пути и ищем ячейку у которой есть
                // зелёный проход (count == 0). После чего расчитываем до той ячейки кратчайший маршрут
                // и перемещаемся туда.

                coordinate_t back_trace = my_coordinate;

                back_bearing = my_bearing;

                do {
                    back_bearing += turn_direction;
                    back_bearing &= TURN_MASK;
                    data.path[data.pathlength++] = turn_direction;
                    simplifyPath();
                    do {
                        maze_count[back_trace.north*MAZE_SIDE + back_trace.east][back_bearing] += 1;
                        update_coordinate(&back_trace, (bearing_dir_t)back_bearing);
                        maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+back) & TURN_MASK] += 1;
                        current_cell_walls = maze[back_trace.north * MAZE_SIDE + back_trace.east];
                        current_cell_walls >>= back_bearing;     // 0000 0000 000a aaab
                        current_cell_walls &= 0x0f;
                    } while ((current_cell_walls & (west_wall | east_wall)) == (west_wall | east_wall));
                    if (((current_cell_walls & west_wall) == 0) && (
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][((back_bearing + left) & TURN_MASK)] == 0))
                        do_prediction(back_trace, (bearing_dir_t)((back_bearing + left) & TURN_MASK));
                    if (((current_cell_walls & east_wall) == 0) && (
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][((back_bearing + right) & TURN_MASK)] == 0))
                        do_prediction(back_trace, (bearing_dir_t)((back_bearing + right) & TURN_MASK));
                    if (((current_cell_walls & north_wall) == 0) && (
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][back_bearing] == 0))
                        do_prediction(back_trace, (bearing_dir_t) back_bearing);

                    count_green = 0; count_yellow = 0; count_red = 0;
                    unavailable = current_cell_walls;
                    for (ii = 0; ii < 4; ii++) {
                        if (unavailable & (1 << ii)) {
                            maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+ii) & TURN_MASK] = 3;
                            continue; // если есть стена, то эту сторону пропускаем
                        }
                        switch (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+ii) & TURN_MASK]) {
                        case 0: count_green++;  break;
                        case 1: count_yellow++; break;
                        default: count_red++;    break;
                        }
                    }
                    if (count_yellow == 3) skiplevel = 0; // есть одиночная нить пересекающая узел - возвращаемся
                    else if (count_green)  break;  // есть зелёные проходы - конец отката.
                    else if (count_yellow) skiplevel = 2;  // оставить все не красные проходы
                    else goto save_map;   // если нет ни зелёного, ни желтого прохода - больше идти некуда.

                    if (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+left) & TURN_MASK] >= skiplevel) unavailable |= west_wall;
                    if (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+right) & TURN_MASK] >= skiplevel) unavailable |= east_wall;
                    if (maze_count[back_trace.north*MAZE_SIDE + back_trace.east][(back_bearing+straight) & TURN_MASK] >= skiplevel) unavailable |= north_wall;
                    turn_direction = SelectTurn(unavailable);
                } while(1);
                // Телепортируемся
                draw_maze(my_coordinate, &global_path);
                LaunchPad_Output(BLUE);
                search_way_simple(my_coordinate, back_trace, my_bearing, 0);
                for (ii = 0; ii < *back_path.pathlength; ii++) {
                    make_turn((rotation_dir_t)back_path.path[ii]);
                    my_bearing += back_path.path[ii];
                    my_bearing &= TURN_MASK;
                    steps = make_step();
                    while (steps-- > 1) {
                        update_coordinate(&my_coordinate, my_bearing);
                        draw_maze(my_coordinate, &global_path);
                        delay_us(1000000);
                    }
                    update_coordinate(&my_coordinate, my_bearing);
                    current_cell_walls = get_walls();
                    update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
                    draw_maze(my_coordinate, &global_path);
                    //                    if (current_cell_walls == (west_wall | north_wall | east_wall)) {
                    //                        if (get_color() == red) break;
                    //                    }
                    delay_us(1000000);
                }
                //                my_bearing = make_turn((rotation_dir_t)((back_bearing - my_bearing) & TURN_MASK));
                current_cell_walls = get_walls();
            } else {
                LaunchPad_Output(0);
                break;
            }
        } while (1);


        make_turn(turn_direction);
        my_bearing += turn_direction;
        my_bearing &= TURN_MASK;
        //        data.path[data.pathlength] = turn_direction;
        data.path[data.pathlength] = (my_bearing - back_bearing) & TURN_MASK;
        data.pathlength++;
        back_bearing = my_bearing;
        // You should check to make sure that the path_length does not
        // exceed the bounds of the array.
        if (data.pathlength >= MAX_PATH_LENGTH) {
            squareXY(0,0, 127, 63, 1);
            update_display();
            putstr(0, 3, "MAX PATH LENGTH", 1);
            //            Motor_Speed(0, 0);
            //            speed  = 0;
            //            where_am_i = old_ret;
            return ;//1;
        }

        // Simplify the learned path.
        simplifyPath();


    } while (1);




save_map:
    search_way_simple(my_coordinate, start, my_bearing, 0);
    for (ii = 0; ii < *back_path.pathlength; ii++) {
        make_turn((rotation_dir_t)back_path.path[ii]);
        my_bearing +=  back_path.path[ii];
        my_bearing &= TURN_MASK;
        steps = make_step();
        while (steps--) {
            update_coordinate(&my_coordinate, my_bearing);
            draw_maze(my_coordinate, &global_path);
            if (steps) {
                current_cell_walls = (east_wall | west_wall);
                update_cell_walls(my_coordinate, my_bearing, current_cell_walls);
                delay_us(1000000);
            }
        }
        draw_maze(my_coordinate, &global_path);
        delay_us(1000000);
    }

    search_way_simple(start, finish, (bearing_dir_t)((data.sq_init & 0xF0000) >> 16), 0);
    copy_data_dma(backpath_turn+1, data.path, backpath_size-1);
    data.pathlength = backpath_size-1;
    while(dma_copy_busy) WaitForInterrupt();
    draw_maze(my_coordinate, &global_path);
    LaunchPad_Output(GREEN);
    return;
}
#endif
/*
 * Расчет кратчайшего пути по лабиринту по алгоритму Дейкстры.
 */

#ifdef SORTED_QUEUE
static unsigned int sort_idx[QUEUE_SIZE], sort_tail=0;

static unsigned int insert_val(unsigned int val, unsigned int idx) {
    unsigned int i;

    if (++sort_tail > (QUEUE_SIZE-1)) return 1;
    for (i = sort_tail-1; i > 0; i--) {
        if(flood[sort_idx[i-1]] < val) {
            sort_idx[i] = sort_idx[i-1];
        } else break;
    }
    sort_idx[i] = idx;
    return 0;
}

static unsigned int extract_val(void) {
    if (sort_tail)  sort_tail--;
    return sort_idx[sort_tail];
}

static void init_queue(void) {
    sort_tail = 0;
}
#else
#define insert_val(x, y)  floodqueue[queue_wr++] = y; queue_wr &= QUEUE_MASK;
#define extract_val()     floodqueue[queue_rd++]; queue_rd &= QUEUE_MASK;
#define sort_tail         queue_rd != queue_wr
unsigned int floodqueue[QUEUE_SIZE], queue_wr = 0, queue_rd = 0;
#endif

const unsigned int cost_table[] = {0, 424, 749, 1024, 1265, 1484, 1685, 1872, 2048, 2214, 2372, 2523, 2668, 2807, 2941, 3072, 3198};
//const unsigned int cost_table[] = {0, 300, 600, 900, 1200, 1500, 1800, 2100, 2400, 2700, 3000, 3300, 3600, 3900, 4200, 4500, 4800};

struct {
    unsigned int north;
    unsigned int east;
    unsigned int south;
    unsigned int west;
} wall;

unsigned int search_way_simple(coordinate_t start, coordinate_t finish,
                               bearing_dir_t prev_bearing, unsigned int open){
    unsigned int distance = 0, pos_idx, cost = 0;
    unsigned int next_virtual_cell, ii;
    coordinate_t ptr = finish;

    fill_data32_dma(0xffffffff, flood, MAZE_SIDE*MAZE_SIDE*2);

    if (open) {
        wall.north = 0x10;
        wall.east  = 0x20;
        wall.south = 0x40;
        wall.west  = 0x80;
    } else { // closed maze:
        wall.north = 0x01;
        wall.east  = 0x02;
        wall.south = 0x04;
        wall.west  = 0x08;
    }

    pos_idx = (ptr.north*MAZE_SIDE+ptr.east)*2;

    while (dma_copy_busy) WaitForInterrupt();
    flood[pos_idx] = distance;
    flood[pos_idx+1] = distance;
    if (insert_val(distance, pos_idx)) LaunchPad_Output(RED);
    if (insert_val(distance, pos_idx+1)) LaunchPad_Output(RED);

    while(sort_tail) {
        pos_idx = extract_val();
        if (pos_idx > (MAZE_SIDE*MAZE_SIDE*2)-1) {
            LaunchPad_LED(1);
            continue;
        }

        if (pos_idx & 0x01) {
            next_virtual_cell = 1;
            distance = flood[pos_idx]+cost_table[next_virtual_cell];
            while (((maze[(pos_idx+2*(next_virtual_cell-1))/2] & wall.east) == 0) && (flood[(pos_idx+2*next_virtual_cell)] > distance)) {
                flood[(pos_idx+2*next_virtual_cell)] = distance;
                if (insert_val(distance, pos_idx + 2*next_virtual_cell)) LaunchPad_Output(RED);
                next_virtual_cell++;
                distance = flood[pos_idx]+cost_table[next_virtual_cell];
            }
            next_virtual_cell = 1;
            distance = flood[pos_idx]+cost_table[next_virtual_cell];
            while (((maze[(pos_idx-2*(next_virtual_cell-1))/2] & wall.west) == 0) && (flood[(pos_idx-2*next_virtual_cell)] > distance)) {
                flood[(pos_idx-2*next_virtual_cell)] = distance;
                if (insert_val(distance, pos_idx - 2*next_virtual_cell)) LaunchPad_Output(RED);
                next_virtual_cell++;
                distance = flood[pos_idx]+cost_table[next_virtual_cell];
            }
        } else {
            next_virtual_cell = 1;
            distance = flood[pos_idx]+cost_table[next_virtual_cell];
            while (((maze[(pos_idx + MAZE_SIDE*2*(next_virtual_cell-1))/2] & wall.north) == 0) && (flood[pos_idx + MAZE_SIDE*2*next_virtual_cell] > distance)) {
                flood[pos_idx + MAZE_SIDE*2*next_virtual_cell] = distance;
                if (insert_val(distance, pos_idx + MAZE_SIDE*2*next_virtual_cell)) LaunchPad_Output(RED);
                next_virtual_cell++;
                distance = flood[pos_idx]+cost_table[next_virtual_cell];
            }
            next_virtual_cell = 1;
            distance = flood[pos_idx]+cost_table[next_virtual_cell];
            while (((maze[(pos_idx - MAZE_SIDE*2*(next_virtual_cell-1))/2] & wall.south) == 0) && (flood[pos_idx - MAZE_SIDE*2*next_virtual_cell] > distance)) {
                flood[pos_idx - MAZE_SIDE*2*next_virtual_cell] = distance;
                if (insert_val(distance, pos_idx - MAZE_SIDE*2*next_virtual_cell)) LaunchPad_Output(RED);
                next_virtual_cell++;
                distance = flood[pos_idx]+cost_table[next_virtual_cell];
            }
        }
        distance = flood[pos_idx]+data.turncost;
        if (flood[pos_idx ^ 0x01] > distance) {
            flood[pos_idx ^ 0x01] = distance;
            if (insert_val(distance, pos_idx ^ 0x01)) LaunchPad_Output(RED);
        }
    }


    // Теперь скатываемся вниз для создания кратчайшего пути
    // от старта к финишу.

    *(back_path.pathlength) = 0; // указатель пути на начало
    pos_idx = (start.north*MAZE_SIDE + start.east)*2;  // начинаем со стартовой точки.
    cost =  flood[pos_idx];  // стоимость маршрута
    if (cost == 0xffffffff) return cost; // маршрут отсутствует.

    while (flood[pos_idx]) {
        if (pos_idx & 0x01) { // east or west
//            east:
            next_virtual_cell = 0;
            while (((((pos_idx+2*(next_virtual_cell))/2)%MAZE_SIDE) < (maxX - 1)) && ((maze[(pos_idx+2*(next_virtual_cell))/2] & wall.east) == 0))  {
                next_virtual_cell++;
                if (flood[pos_idx] - flood[pos_idx + 2*next_virtual_cell] == cost_table[next_virtual_cell]) {
                    for (ii=0; ii < next_virtual_cell; ii++) {
                        if (maze[pos_idx/2] != 0x55) back_path.path[(*back_path.pathlength)++] = (east - prev_bearing) & TURN_MASK;
                        prev_bearing = east;
                        pos_idx += 2;
                    }
                    next_virtual_cell = 0;
                    continue;
                }
            }
//            west:
            next_virtual_cell = 0;
            while (((((pos_idx-2*(next_virtual_cell))/2)%MAZE_SIDE) > 0) && ((maze[(pos_idx-2*(next_virtual_cell))/2] & wall.west) == 0))  {
                next_virtual_cell++;
                if (flood[pos_idx] - flood[pos_idx - 2*next_virtual_cell] == cost_table[next_virtual_cell]) {
                    for (ii=0; ii < next_virtual_cell; ii++) {
                        if (maze[pos_idx/2] != 0x55) back_path.path[(*back_path.pathlength)++] = (west - prev_bearing) & TURN_MASK;
                        prev_bearing = west;
                        pos_idx -= 2;
                    }
                    next_virtual_cell = 0;
                    continue;
                }
            }
        } else { // north or south
//            north:
            next_virtual_cell = 0;
            while ((pos_idx/2+MAZE_SIDE*(next_virtual_cell) <= MAZE_SIDE*(MAZE_SIDE - 1)) && ((maze[pos_idx/2+MAZE_SIDE*(next_virtual_cell)] & wall.north) == 0)) {
                next_virtual_cell++;
                if (flood[pos_idx] - flood[pos_idx+MAZE_SIDE*2*next_virtual_cell] == cost_table[next_virtual_cell]) {
                    for (ii=0; ii < next_virtual_cell; ii++) {
                        if (maze[pos_idx/2] != 0xAA) back_path.path[(*back_path.pathlength)++] = (north - prev_bearing) & TURN_MASK;
                        prev_bearing = north;
                        pos_idx += MAZE_SIDE*2;
                    }
                    next_virtual_cell = 0;
                    continue;
                }
            }
//            south:
            next_virtual_cell = 0;
            while ((pos_idx/2 >= MAZE_SIDE*(next_virtual_cell)) && ((maze[pos_idx/2-MAZE_SIDE*(next_virtual_cell)] & wall.south) == 0)) {
                next_virtual_cell++;
                if (flood[pos_idx] - flood[pos_idx-MAZE_SIDE*2*next_virtual_cell] == cost_table[next_virtual_cell]) {
                    for (ii=0; ii < next_virtual_cell; ii++) {
                        if (maze[pos_idx/2] != 0xAA) back_path.path[(*back_path.pathlength)++] = (south - prev_bearing) & TURN_MASK;
                        prev_bearing = south;
                        pos_idx -= MAZE_SIDE*2;
                    }
                    continue;
                }
            }
        }
        if (flood[pos_idx] - flood[pos_idx ^ 0x01] == data.turncost) {
            pos_idx ^= 0x01;
            continue;
        }
    }
    return cost;
}


/*
 * Проверка маршрута по лабиринту от старта до моей позиции
 * на наличие неисследованных ячеек, которые могут закрыть путь.
 */
unsigned int test_route(coordinate_t * my_place, bearing_dir_t my_direction) {
    coordinate_t last_unknown;
    unsigned int ii, has_unknown = 0, direction = my_direction;

    for (ii = 0; ii < data.pathlength; ii++) {
        do {
            unsigned int maze_cell;
            update_coordinate(my_place, (bearing_dir_t)direction);
            maze_cell = maze[my_place->north*MAZE_SIDE + my_place->east];
            if (((maze_cell & 0xF0) >> 4) != (maze_cell & 0x0F)) {
                last_unknown = *my_place;
                has_unknown =1;
            }
            if ((maze_cell != 0x55) && (maze_cell != 0xAA)) break;
        } while (1);
        direction += data.path[ii];
        direction &= TURN_MASK;
    }
    *my_place = last_unknown;
    return has_unknown;
}

void flood_maze(unsigned int to_finish_cell) {
    unsigned int pos_idx, distance, ii, jj;

    fill_data32_dma(0xFFFFFFFF, flood, sizeof(flood)/sizeof(flood[0]));
    init_queue();

    wall.north = 0x10;
    wall.east  = 0x20;
    wall.south = 0x40;
    wall.west  = 0x80;

    while (dma_copy_busy) continue;
    if (to_finish_cell) {
        for (ii=7; ii <= 8; ii++) {
            for (jj = 7; jj <= 8; jj++) {
                flood[ii*maxX + jj*2] = 0;      insert_val(0, ii*maxX + jj*2);
                flood[ii*maxX + jj*2 + 1] = 0;  insert_val(0, ii*maxX + jj*2 + 1);
            }
        }
    } else {
        flood[0] = 0; insert_val(0, 0);
        flood[1] = 0; insert_val(0, 1);
    }
    while(sort_tail) {
        pos_idx = extract_val();
        if (pos_idx > (MAZE_SIDE*MAZE_SIDE*2)-1) {
            LaunchPad_LED(1);
            continue;
        }

        if (pos_idx & 0x01) {
            distance = flood[pos_idx] + data.stepcost;
            if (((maze[pos_idx/2] & wall.east) == 0) && (flood[pos_idx+2] > distance)) {
                flood[(pos_idx+2)] = distance;
                if (insert_val(distance, pos_idx + 2)) LaunchPad_Output(RED);
                distance = flood[pos_idx] + data.stepcost;
            }
            distance = flood[pos_idx] +data.stepcost;
            if (((maze[pos_idx/2] & wall.west) == 0) && (flood[pos_idx-2] > distance)) {
                flood[(pos_idx-2)] = distance;
                if (insert_val(distance, pos_idx - 2)) LaunchPad_Output(RED);
                distance = flood[pos_idx] + data.stepcost;
            }
        } else {
            distance = flood[pos_idx] + data.stepcost;
            if (((maze[pos_idx/2] & wall.north) == 0) && (flood[pos_idx + MAZE_SIDE*2] > distance)) {
                flood[pos_idx + MAZE_SIDE*2] = distance;
                if (insert_val(distance, pos_idx + MAZE_SIDE*2)) LaunchPad_Output(RED);
                distance = flood[pos_idx] + data.stepcost;
            }
            distance = flood[pos_idx] + data.stepcost;
            if (((maze[pos_idx/2] & wall.south) == 0) && (flood[pos_idx - MAZE_SIDE*2] > distance)) {
                flood[pos_idx - MAZE_SIDE*2] = distance;
                if (insert_val(distance, pos_idx - MAZE_SIDE*2)) LaunchPad_Output(RED);
                distance = flood[pos_idx] + data.stepcost;
            }
        }
        distance = flood[pos_idx]+data.turncost;
        if (flood[pos_idx ^ 0x01] > distance) {
            flood[pos_idx ^ 0x01] = distance;
            if (insert_val(distance, pos_idx ^ 0x01)) LaunchPad_Output(RED);
        }
    }

}

