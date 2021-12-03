/*
 * Maze.c
 *
 *  Created on: 24 февр. 2019 г.
 *      Author: wl
 */
#include "msp.h"
#include "stdint.h"
#include "Motor.h"
#include "SPI_EEProm.h"
#include "main.h"
#include "display.h"
#include "LaunchPad.h"
#include "color_sensor.h"
#include "Reflectance.h"
#include "Tachometer.h"
#include "keyboard.h"
#include "rand.h"
#include "math.h"
#include "BumpInt.h"
#include "Clock.h"
#include "Blinker.h"
#include "dma.h"
#include "Logging.h"
#include "Timer32.h"

#ifdef WITH_BGX
	#include "UART1.h"
#endif


#define MAX_PATH_LENGTH (MAX_MAP_SIZE * 2)
rotation_dir_t path[MAX_PATH_LENGTH];
int length[MAX_PATH_LENGTH];

#define CURRENT_DISTANCE	((LeftSteps + RightSteps) * 11 / 36)  // 220mm per 360 tick of two wheels.

map_cell_t map[MAX_MAP_SIZE];
uint8_t	where_am_i = 0, old_ret = 0;

const unsigned int turn_sequence[][3] = {
				{RIGHT_MASK, STRAIGHT_MASK, LEFT_MASK},
				{LEFT_MASK, STRAIGHT_MASK, RIGHT_MASK},
				{RIGHT_MASK, LEFT_MASK, STRAIGHT_MASK},
				{LEFT_MASK, RIGHT_MASK, STRAIGHT_MASK},
				{STRAIGHT_MASK, RIGHT_MASK, LEFT_MASK},
				{STRAIGHT_MASK, LEFT_MASK, RIGHT_MASK},
};

typedef enum {
	slow,
	fast
} speed_t;

volatile uint8_t CollisionFlag = 0, CollisionData = 0;
int speed = 0;

void CollisionHandler(uint8_t bump_data) {
	if (!CollisionFlag) {
		Motor_Stop();
		CollisionData |= bump_data;
	}
	CollisionFlag = 1;
	CollisionData |= bump_data;
}


// функция движения по сегменту.
// если задана быстрая скорость, то робот движеся со скоростью maxmotor на участке distance, затем maxspeed.
// если задана медленная скорость, то весь сегмент движется со скоростью maxspeed.
// в пределах guarddist игнорируется пропадание линии и срабатывание боковых сенсоров.
void run_segment(speed_t runspeed, unsigned int distance) {
    unsigned int activ_sensor_count, last_sensor, first_sensor, photo_mask;
    unsigned int blind, blindcount = 0, i, finishcount = 0, slowdistance, ignoredistance; 
    int track_error, de_dt, sigma_error = 0, prev_track_error=0, left_speed, right_speed, result, maxspeed;
	uint8_t backlight = 0;

    old_ret = where_am_i;
    if (where_am_i) where_am_i = Segment;

    if (runspeed == slow) maxspeed = data.turnspeed, backlight = BK_LEFT | BK_RGHT;
    else				  maxspeed = data.maxspeed, backlight = FR_LEFT | FR_RGHT;
#ifdef BLINKER_SEGMENT
    Blinker_Output(backlight);
#else
    (void)backlight;
#endif
    slowdistance = CURRENT_DISTANCE + distance;
    ignoredistance = CURRENT_DISTANCE + data.guarddist;
	
    while(1) {
        if (photo_data_ready ) {
            if (CollisionFlag) return;
            photo_data_ready  = 0;
            data_log(where_am_i << 8 | current_sensor, 1);
            track_error = 0; blind = 1; activ_sensor_count = 0; last_sensor = 256; first_sensor = 256; photo_mask = 0;

            photo_mask = current_sensor;
            for (i=0; i<8; i++) {
                if (photo_mask & 0x01u) {
                    blind = 0;
                    activ_sensor_count++;
                    last_sensor = i;
                    if (first_sensor == 256) first_sensor = i;
                }
                photo_mask >>= 1;
            }

            photo_mask = current_sensor;
            if (blind)      LaunchPad_LED(1);
            else            LaunchPad_LED(0);

//  *  Теперь исходя из данных фотосенсоров пытаемся установить координаты линии.
//  *  0 - центр, края +/- 332.
//  * Если нет сигнала вообще - тупик - выход из подпрограммы.
        
            if (blind && CURRENT_DISTANCE >= ignoredistance) {
                if (++blindcount > 2) {
#ifdef BLINKER_SEGMENT
                    Blinker_Output(0);
#endif
                    where_am_i = old_ret;
                    return; // нет линии - тупик
                }
                track_error = prev_track_error;
            } else {
                blindcount = 0;
                if (CURRENT_DISTANCE >= ignoredistance) {
                    if ((photo_mask & 0x81) && ((last_sensor - first_sensor) == (activ_sensor_count - 1))) {
                        // есть срабатывание боковых датчиков - поворот или перекрёсток
                        if (++finishcount > 1) {
#ifdef BLINKER_SEGMENT
                            Blinker_Output(0);
#endif
                            where_am_i = old_ret;
                            return;
                        }										// Ждём повторного срабатывания и выходим
                    } else {
                        finishcount = 0;		// Если нет повторного - показалось.
                    }
                }
                track_error = line_error(first_sensor, last_sensor);
            }
            
// Теперь имеем значение ошибки/смещения и дифференциальное значение ошибки

            de_dt = data.k_diff*(track_error - prev_track_error);

            sigma_error += (track_error * data.div_sigma) >> 10;
            if (sigma_error > 2048) sigma_error = 2048;
            if (sigma_error < -2048) sigma_error = -2048;

            prev_track_error = track_error;

            result = data.k_error*track_error + de_dt + sigma_error;

            if (result < -14999) result = -14999;
            else if (result > 14999) result = 14999;

            if (CURRENT_DISTANCE >= slowdistance) {
                if (maxspeed > data.turnspeed) maxspeed -= data.acceleration;
                else 	maxspeed = data.turnspeed;
                backlight = BK_LEFT | BK_RGHT;
#ifdef BLINKER_SEGMENT
                Blinker_Output(backlight);
#endif
            }
            if ((speed + (ABS(result)*speed)/8192) > data.maxspeed) {
                speed = data.maxspeed - (ABS(result)*speed)/8192;
                if (speed < data.minspeed) speed = data.minspeed;
            }

            if ((ABS(result)) < (data.on_way*data.k_error)) {
                if ((speed += data.acceleration) > maxspeed) speed = maxspeed;
                if (speed < 1500) speed = 1500;
            }
            left_speed = speed+(result*speed)/8192; // speed*(1+result/nom_speed)
            right_speed = speed-(result*speed)/8192;

            if (left_speed > data.maxmotor) left_speed = data.maxmotor;
            else if (left_speed < -data.maxmotor) left_speed = -data.maxmotor;

            if (right_speed > data.maxmotor) right_speed = data.maxmotor;
            else if (right_speed < -data.maxmotor) right_speed = -data.maxmotor;

            Motor_Speed(left_speed, right_speed);
        }
    }
}


// 180grad ~ 800
// 180grad ~ 745
// x*745/180
// x*149/36
//#define DEGREE(x)  ((x*40+4)/9)
#define DEGREE(x)  ((x*149)/36)

// Turns according to the parameter dir
// returns 0 if OK, 1 if ERROR

unsigned int turn(rotation_dir_t dir) {
    static rotation_dir_t last_turn = left;
    unsigned int  turn_dir, count = 0, degree = 90, turnspeed = 0;
    unsigned int  last_status = 0, photo_sensor;
    int stop_difference, fail_difference, slow_difference;

    old_ret = where_am_i;
    if (where_am_i) where_am_i = Turn;

    turn_dir = dir;
    if (dir == back) {
        degree = 180;
        turn_dir = (last_turn == left) ? right : left;
    }

    switch (turn_dir) {
        case left:      // turn left
            speed = 0;
            last_turn = left;
            stop_difference = (RightSteps - LeftSteps) + ((degree == 180) ? DEGREE(135) : DEGREE(45));
            slow_difference = (RightSteps - LeftSteps) + ((degree == 180) ? DEGREE(135) : DEGREE(60));
            fail_difference = (RightSteps - LeftSteps) + ((degree == 180) ? DEGREE(360) : DEGREE(135));
            do {
//				Motor_Speed(-data.turnspeed, data.turnspeed);
                count = 0;
                while(count < 2) {
                    if (photo_data_ready ) {
                        photo_data_ready  = 0;
                        photo_sensor = current_sensor;
                        data_log(where_am_i << 8 | current_sensor, 1);

                        if ((RightSteps - LeftSteps) < slow_difference) {
                            if ((turnspeed += data.acceleration) < 1500) turnspeed = 1500;
                            if (turnspeed > data.maxmotor) turnspeed = data.maxmotor;
                        } else {
                            if ((turnspeed -= data.acceleration) < data.turnspeed) turnspeed = data.turnspeed;
                        }
                        Motor_Speed(-turnspeed, turnspeed);

                        if ((photo_sensor & (1 << 5)) ^ last_status) count++;
                        last_status = photo_sensor & (1 << 5);
                        if ((RightSteps - LeftSteps) > fail_difference) {
                            where_am_i = old_ret;
                            return 1;
                        }
                    }
                    if (CollisionFlag) return 1;
                }
            } while ((RightSteps - LeftSteps) < stop_difference);
            break;

        case right:  // Turn right.
            speed = 0;
            last_turn = right;
            stop_difference = (LeftSteps - RightSteps) + ((degree == 180) ? DEGREE(135) : DEGREE(45));
            slow_difference = (LeftSteps - RightSteps) + ((degree == 180) ? DEGREE(135) : DEGREE(60));
            fail_difference = (LeftSteps - RightSteps) + ((degree == 180) ? DEGREE(360) : DEGREE(135));
            do {
//				Motor_Speed(data.turnspeed, -data.turnspeed);
                count = 0;
                while(count < 2) {
                    if (photo_data_ready ) {
                        photo_data_ready  = 0;
                        photo_sensor = current_sensor;
                        data_log(where_am_i << 8 | current_sensor, 1);

                        if ((LeftSteps - RightSteps) < slow_difference) {
                            if ((turnspeed += data.acceleration) < 1500) turnspeed = 1500;
                            if (turnspeed > data.maxmotor) turnspeed = data.maxmotor;
                        } else {
                            if ((turnspeed -= data.acceleration) < data.turnspeed) turnspeed = data.turnspeed;
                        }
                        Motor_Speed(turnspeed, -turnspeed);

                        if ((photo_sensor & (1 << 2)) ^ last_status) count++;
                        last_status = photo_sensor & (1 << 2);
                        if ((LeftSteps - RightSteps) > fail_difference) {
                            where_am_i = old_ret;
                            return 1;
                        }
                    }
                    if (CollisionFlag) return 1;
                }
            } while ((LeftSteps - RightSteps) < stop_difference);
            break;

        default:
            break;
    }
    //    Motor_Speed(0, 0);
    where_am_i = old_ret;
    return 0;
}

unsigned int SpeedTurn(rotation_dir_t dir) {
    unsigned int  turn_dir;
    int stop_difference;

    old_ret = where_am_i;
    if (where_am_i) where_am_i = Turn;

    turn_dir = dir;

    switch (turn_dir) {
        case left:      // turn left
            stop_difference = RightSteps + DEGREE(90);
            do {
                Motor_Speed(0, speed);
                if (photo_data_ready ) {
                    photo_data_ready  = 0;
                    data_log(where_am_i << 8 | current_sensor, 1);
                }
                if (CollisionFlag) return 1;
            } while (RightSteps < stop_difference);
            break;

        case right:  // Turn right.
            stop_difference = LeftSteps + DEGREE(90);
            do {
                Motor_Speed(speed, 0);
                if (photo_data_ready ) {
                    photo_data_ready  = 0;
                    data_log(where_am_i << 8 | current_sensor, 1);
                }
                if (CollisionFlag) return 1;
            } while (LeftSteps < stop_difference);
            break;

        default:
            break;
    }
    //    Motor_Speed(0, 0);
    where_am_i = old_ret;
    return 0;
}

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variable available_directions which indicate
// whether there is an exit in each of the three directions. If no - turn back
rotation_dir_t SelectTurn(unsigned int available_directions, const unsigned int * sequence, unsigned int skipcount) {
    unsigned int  i;
    if (available_directions) {
        for (i = 0; i < 3; i++) {
            if (available_directions & sequence[i]) {
                if (skipcount-- == 0) break;
            }
        }
        switch (sequence[i]) {
            case STRAIGHT_MASK:
                return straight;
            case LEFT_MASK:
                return left;
            case RIGHT_MASK:
                return right;
            default:
                return back;
        }
    }
    return back;
}

// simplifyPath analyzes the path[] array and reduces all the
// back turns. For example: Right turn + Back + Right turn = Straight.
static void simplifyPath(void) {
    int total_angle = straight;
    int i;

    // only simplify the path if the second-to-last turn was a "Back"
    if(data.pathlength < 3 || path[data.pathlength - 2] != back) return;


    for(i = 1; i <= 3; i++) total_angle += path[data.pathlength - i];
    total_angle &= TURN_MASK;
    path[data.pathlength - 3] = (rotation_dir_t)total_angle;
    // The path is now two steps shorter.
    data.pathlength -= 2;
}

void update_coordinate (coordinate_t *current_coordinate, unsigned int length, bearing_dir_t moving_direction) {
	switch (moving_direction) {
		case north:	current_coordinate->north += length;	break;
		case south:	current_coordinate->north -= length;	break;
		case east:	current_coordinate->east += length;		break;
		case west:	current_coordinate->east -= length;		break;
	}
}


// The solveMaze() function works by applying a Luc-Tremo algorithm.
// It records each turn it makes. Turn selection depends on strategy
// selected by variable data.lefthand (values from 0 to 5, otherwise uses random), 
// excluding branches where robot already was checked.
// Afterwards, the recorded path is simplified by removing dead ends.
// When there is recorded path, robot first follow this path.
// In explore mode recorded path will be deleted and robot don't stop in end cell, but record.
unsigned int solveMaze(unsigned int explore_mode) {
	unsigned int maze_entrance = 1, available, photo_sensor, step=0, play = 0, i, blind;
	unsigned int /*loop=0, fork=1, fork_count, mask, */ skip=0, skiplevel, maxspeed;
	unsigned int last_index = 0, current_index, count_green, count_yellow, count_red, found_red = 0;
	unsigned int expect_index = UNKNOWN, turn_index;
//	uint32_t *data_ptr;
	rotation_dir_t   turn_direction = straight;
	int		move_direction = north;
	coordinate_t my_coordinate = {0, 0};
	unsigned int max_map_cell = 0;
	int segment_length, start_position;

	where_am_i = Solve;
    data_log_init();

	CollisionFlag = 0;
	BumpInt_Init(&CollisionHandler);
	LaunchPad_Output(GREEN);
	if (explore_mode)   maxspeed = data.turnspeed;
	else                maxspeed = data.maxspeed;
	
// Проезжаем вперед на половину корпуса вслепую
	segment_length= CURRENT_DISTANCE  + 100;// - data.sensor_offset;
	speed = 0;
	while (CURRENT_DISTANCE < segment_length) {
	    if (photo_data_ready) {
	        photo_data_ready = 0;
	        data_log(where_am_i << 8 | current_sensor, 1);

	        //				if (speed < data.maxspeed) speed += data.acceleration;
	        if ((speed += data.acceleration) > maxspeed) speed = maxspeed;
	        if (speed < 1500) speed = 1500;
	        Motor_Speed(speed, speed);
	    }
	}

full_restart:	
	max_map_cell = 0;
	found_red = 0;
	last_index = 0;
	maze_entrance = 1;
	
	map[0].coordinate.north = my_coordinate.north;
	map[0].coordinate.east  = my_coordinate.east;
	map[0].pass_count[0] = 1;
	map[0].pass_count[1] = 4;
	map[0].pass_count[2] = 4;
	map[0].pass_count[3] = 4;
	map[0].node_link[0] = 0;
	map[0].node_link[1] = UNKNOWN;
	map[0].node_link[2] = UNKNOWN;
	map[0].node_link[3] = UNKNOWN;
	max_map_cell++;
	
	if (explore_mode) {
		data.pathlength = 0;
		data.green_cell_nr = 0;
		play = 0;
	} else {
		if (data.pathlength > 0) play = 1;
	    copy_data_dma(data.path, path, data.pathlength);
	    copy_data_dma((uint8_t *)data.length, (uint8_t *)length, data.pathlength * 4);

//		for (i=0; i<data.pathlength; i++ ){
//				path[i] = (rotation_dir_t) data.path[i];
//				length[i] =  data.length[i];
//		}
	}
//	loop = data.loop;
	
// Начальный вход в лабиринт: движемся вперед до появления линии
//	Motor_Speed(data.maxspeed, data.maxspeed);
// Проезжаем вперед на половину корпуса вслепую
//		segment_length= CURRENT_DISTANCE  + 100 - data.sensor_offset;
//		Motor_Speed(data.turnspeed, data.turnspeed);
//		while (CURRENT_DISTANCE < segment_length) continue;
//	photo_sensor = 0;
//	do {
//			if (photo_data_ready ) {
//					photo_data_ready  = 0;
//					photo_sensor = current_sensor;
//			}
//	} while (photo_sensor == 0 && CollisionFlag == 0);


	do {

		LaunchPad_Output(0);
		// Navigate current line segment

		if (maze_entrance)
			start_position = CURRENT_DISTANCE;
		else
			start_position = CURRENT_DISTANCE - data.sensor_offset; // Добавляем расстояние от оси до сенсоров.

		if (play) {
			if (path[step] == straight) {
				run_segment(fast, length[step]);
			} else {
				if (length[step] > data.brake_path) {
					run_segment(fast, length[step] - data.brake_path);
				} else {
					run_segment(slow, 0);
				}
			}
		} else {
			run_segment(slow, 0);
		};

		if (CollisionFlag) {
		    CollisionFlag = 0;
			squareXY(0,0, 127, 63, 1);
			update_display();
			putstr(3, 3, "BUMPER HIT", 1);
			where_am_i = old_ret;
			return 1;
		}
		LaunchPad_Output(BLUE);
		maze_entrance = 0;
		segment_length = CURRENT_DISTANCE - start_position;
		start_position = CURRENT_DISTANCE;
		update_coordinate(&my_coordinate, segment_length, (bearing_dir_t)move_direction);

		available = 0;
		photo_sensor = current_sensor;

		// Check for left and right exits.
		if(photo_sensor & (1u << 7)) { available |= LEFT_MASK; }
		if(photo_sensor & (1u << 0)) { available |= RIGHT_MASK; }
		if (where_am_i)  where_am_i = 0x01 | available;

		Motor_Speed(speed, speed);

		while (photo_sensor & ((1u << 0) | (1u << 7))) {
			// Drive straight a bit more, until we are
			// have side sensors active.
			// This should help us better detect if we
			// have left or right segments.
			if (photo_data_ready ) {
			    unsigned int linecount = 0, prev_stat = 0;
				photo_data_ready  = 0;
				photo_sensor = current_sensor;
				data_log(where_am_i << 8 | current_sensor, 1);

				// проверка условия непрерывности линии
				for (i=0; i< 8; i++) {
				    if (prev_stat != (BITBAND_SRAM(photo_sensor, i))) {
		                if (prev_stat = BITBAND_SRAM(photo_sensor, i) == 1) linecount++;
				    }
				}
				// поворот детектируется только если есть одна непрерывная линия
				if (linecount == 1) {
                    if(photo_sensor & (1u << 7)) { available |= LEFT_MASK; }
                    if(photo_sensor & (1u << 0)) { available |= RIGHT_MASK; }
				}
				if (where_am_i) where_am_i |= available;
				if (CollisionFlag) {
		            CollisionFlag = 0;
					squareXY(0,0, 127, 63, 1);
					update_display();
					putstr(3, 3, "BUMPER HIT", 1);
					where_am_i = old_ret;
					return 1;
				}
			}
		}

//		if (available != (LEFT_MASK | RIGHT_MASK)) { // если поворот, то делаем небольшую задержку
		if (1) {
		i=10;
			blind = 0;
			while (i) {
//		    while ((CURRENT_DISTANCE - start_position) < LINE_WIDTH) {
				if (photo_data_ready ) {
					photo_data_ready  = 0;
					photo_sensor = current_sensor;
					if (photo_sensor == 0) blind = 1;
					data_log(where_am_i << 8 | current_sensor, 1);
					i--;
				}
			}
		}

		// now detect if we have straight segment
		if ((photo_sensor & ((1u << 2) | (1u << 3) | (1u << 4) | (1u << 5))) && !blind) {
			available |= STRAIGHT_MASK; 
			if (where_am_i)  where_am_i = 0x01 | available;
		}

        // Теперь выполняем "выравнивание" робота в координатной сетке ортогонального лабиринта
        for (current_index = 0; current_index < max_map_cell; current_index++) { // ищем похожие координаты
/*
*				if ((ABS(my_coordinate.east - map[current_index].coordinate.east)) < data.tolerance) {
*					my_coordinate.east = map[current_index].coordinate.east;
*				}
*				if ((ABS(my_coordinate.north - map[current_index].coordinate.north)) < data.tolerance) {
*					my_coordinate.north = map[current_index].coordinate.north;
*				}
*/
            // И проверяем, были ли мы уже здесь?
            if ((ABS(my_coordinate.east - map[current_index].coordinate.east) < data.tolerance) &&
                    (ABS(my_coordinate.north - map[current_index].coordinate.north) < data.tolerance)) {
                        // я тут был.
                        my_coordinate.east = map[current_index].coordinate.east;
                        my_coordinate.north = map[current_index].coordinate.north;
                        break; // current_index указывает на ячейку карты.
            }
        }
        if (where_am_i) {
            data_log( (my_coordinate.east & 0x000000ff)			| (coord_east_lsb << 8), 1);
            data_log( ((my_coordinate.east & 0x0000ff00) >> 8)	| (coord_east_msb << 8), 1);
            data_log( (my_coordinate.north & 0x000000ff) 		| (coord_north_lsb << 8), 1);
            data_log( ((my_coordinate.north & 0x0000ff00)>> 8)	| (coord_north_msb << 8), 1);
            data_log( (segment_length & 0x000000ff) 			| (segm_length_lsb << 8), 1);
            data_log( ((segment_length & 0x0000ff00)>> 8)		| (segm_length_msb << 8), 1);
            data_log( (current_index & 0xff)                    | (node_idx << 8) , 1);
        }
#ifdef WITH_BGX
        if (STREAM_BIT) {
            char clipboard[12];
            UART1_OutString("\r\nCurrent Index: "); num2str(current_index, clipboard);	UART1_OutString(clipboard);
            if (expect_index != UNKNOWN) {
                UART1_OutString("Expect Index: "); num2str(expect_index, clipboard);	UART1_OutString(clipboard);
            }
            UART1_OutString("\r\nSegment length: "); num2str(segment_length, clipboard);	UART1_OutString(clipboard);
            UART1_OutString("\r\nCoordinate: "); num2str(my_coordinate.east, clipboard);	UART1_OutString(clipboard);
            UART1_OutString(" "); num2str(my_coordinate.north, clipboard);	UART1_OutString(clipboard);
            UART1_OutString("\r\n");
        }
#endif
			
			// нет, мы тут еще не были
			if (current_index == max_map_cell && expect_index == UNKNOWN) {  // новая ячейка
				map[current_index].coordinate.east = my_coordinate.east;
				map[current_index].coordinate.north = my_coordinate.north;
				map[current_index].node_link [(move_direction + back) & TURN_MASK] = last_index;
				map[current_index].node_link [(move_direction+straight) & TURN_MASK] = UNKNOWN;
				map[current_index].node_link [(move_direction + left)  & TURN_MASK] = UNKNOWN;
				map[current_index].node_link [(move_direction + right) & TURN_MASK] = UNKNOWN;

				map[current_index].pass_count[(move_direction + back) & TURN_MASK] = 1;
				map[current_index].pass_count[(move_direction+straight) & TURN_MASK]	=	(available & STRAIGHT_MASK) ? 0 : 4;
				map[current_index].pass_count[(move_direction + left) & TURN_MASK] = 	(available & LEFT_MASK)     ? 0 : 4;
				map[current_index].pass_count[(move_direction + right)& TURN_MASK] = 	(available & RIGHT_MASK)    ? 0 : 4;

				if (++max_map_cell > MAX_MAP_SIZE) {
					squareXY(0,0, 127, 63, 1);
					update_display();
					putstr(0, 3, " OUT OF MEMORY  ", 1);
					max_map_cell = MAX_MAP_SIZE;
				}

				if ( ! available ) {
				// If dead end, check for the ending spot.
				// If field color is red, we have solved the maze.
#ifndef COLOR_SENSOR_ON_BACK
					switch (check_color()) {
						case red: 
#ifdef WITH_BGX
							if (STREAM_BIT) {
								UART1_OutString("Dead end color: Red\r\n");
							}
#endif
							found_red = current_index;
							LaunchPad_Output(RED);
							if (explore_mode) break;
							map[last_index].node_link[move_direction & TURN_MASK] = current_index;
							goto finish;

						// if green we found start position - restart
						// if green field was start - we never will be here again!
						case green:
#ifdef WITH_BGX
							if (STREAM_BIT) {
								UART1_OutString("Dead end color: Green\r\n");
							}
#endif
							LaunchPad_Output(GREEN);
							if (explore_mode) {
								data.green_cell_nr = current_index;
								data.pathlength = 0;
							} else {
								play = 0;
								step = 0;
								data.pathlength = 0;
								turn(back);
								move_direction = north;
								my_coordinate.east = my_coordinate.north = 0;
								goto full_restart;
							}
						// Simply dead end - turn back.
						default:
#ifdef WITH_BGX
							if (STREAM_BIT) {
								UART1_OutString("Dead end color: None\r\n");
							}
#endif
							LaunchPad_Output(0);
							break;
					}
#endif
				}
			} else { // старая ячейка
				if ((expect_index != UNKNOWN) && (expect_index != current_index)) {
					if ((expect_index != UNKNOWN) && data.ignore_coordinate_error) {
						current_index = expect_index;
						my_coordinate.north = map[expect_index].coordinate.north;
						my_coordinate.east  = map[expect_index].coordinate.east;
					} else {
						char number_str[16];
						unsigned int strlen;
						squareXY(0,0, 127, 63, 1);
						update_display();
						putstr(0, 2, "I AM LOST AGAIN", 1);
						putstr(0, 3, "Expect ", 1);
						num2str(expect_index, number_str);
						putstr(8, 3, number_str, 1);
						strlen = num2str(map[expect_index].coordinate.east, number_str);
						putstr(0, 4, number_str, 1);
						putstr(strlen, 4, ",", 1);
						num2str(map[expect_index].coordinate.north, number_str);
						putstr(strlen+1, 4, number_str, 1);
						putstr(0, 5, "Current ", 1);
						num2str(current_index, number_str);
						putstr(8, 5, number_str, 1);
						strlen = num2str(my_coordinate.east, number_str);
						putstr(0, 6, number_str, 1);
						putstr(strlen, 6, ",", 1);
						num2str(my_coordinate.north, number_str);
						putstr(strlen+1, 6, number_str, 1);
						Motor_Stop();
						where_am_i = old_ret;
						return 1;
					}
				}
				if (map[current_index].pass_count[(move_direction + back) & TURN_MASK] < 2) {
						map[current_index].pass_count[(move_direction + back) & TURN_MASK] += 1;
				}
				map[current_index].node_link [(move_direction + back) & TURN_MASK] = last_index;

				count_green = 0; count_yellow = 0; count_red = 0;
				for (i = 0; i < 4; i++) {
					switch (map[current_index].pass_count[i]) {
						case 0:	count_green++; 	break;
						case 1: count_yellow++; break;
						case 2: count_red++;	break;
					}
				}
				if (count_yellow == 3) skiplevel = 0; // есть одиночная нить пересекающая узел - возвращаемся
				else if (count_green)  skiplevel = 1;  // оставить только зелёные проходы
				else if (count_yellow) skiplevel = 2;  // оставить все не красные проходы
				else goto save_map;   // если нет ни зелёного, ни желтого прохода - больше идти некуда.
// ********************************
//   Попробуем так. Будем использовать данные от первоначального анализа узла.
					available = LEFT_MASK | RIGHT_MASK | STRAIGHT_MASK;
// ********************************
				if (map[current_index].pass_count[(move_direction+left) & TURN_MASK] >= skiplevel) available &= ~LEFT_MASK;
				if (map[current_index].pass_count[(move_direction+right) & TURN_MASK] >= skiplevel) available &= ~RIGHT_MASK;
				if (map[current_index].pass_count[(move_direction+straight) & TURN_MASK] >= skiplevel) available &= ~STRAIGHT_MASK;
			}
			map[last_index].node_link[move_direction & TURN_MASK] = current_index;
			if (where_am_i)  where_am_i = 0x01 | available;
#ifdef WITH_BGX
			if (STREAM_BIT) {
				UART1_OutString("Available: ");
				if (available & LEFT_MASK) 			UART1_OutString("Left ");
				if (available & STRAIGHT_MASK)	UART1_OutString("Straight ");
				if (available & RIGHT_MASK) 		UART1_OutString("Right ");
				UART1_OutString("\r\n");
			}
#endif

			if (data.runnumber == 0) {
				show_number(current_index, 0);
				if (available & LEFT_MASK) 			show_arrow(40, 6, left);
				if (available & STRAIGHT_MASK)	show_arrow(56, 6, straight);
				if (available & RIGHT_MASK) 		show_arrow(72, 6, right);
			}
// если нет прямого хода, то всё-равно надо снижать скорость
//			if (available & STRAIGHT_MASK != STRAIGHT_MASK) {
//                speed = data.turnspeed;
//                Motor_Speed(speed, speed);
//            }

      // Intersection identification is complete.
			
//			fork_count = 0, mask = 1 << 0;
//			for (i=0; i < 3; i++) {
//				if (available & mask) fork_count++;
//				mask <<= 1;
//			}
//			if (fork_count > 1) fork <<= 1;
//			if ((loop << 1) & fork) skip = 1;

			turn_index = (data.lefthand < 6) ? data.lefthand : Rand() % 6;
				
			if (play) {
			    turn_direction = path[step++];
			    if (turn_direction == back) break;
			}
			else	turn_direction = SelectTurn(available, turn_sequence[turn_index], skip);
			skip = 0;

			// Make the turn indicated by the path.
#ifdef WITH_BGX
			if (STREAM_BIT) {
				UART1_OutString("Turn ");
				switch (turn_direction) {
					case left:
						UART1_OutString("Left"); break;
					case straight:
						UART1_OutString("Straight"); break;
					case right:
						UART1_OutString("Right"); break;
					case back:
						UART1_OutString("Back"); break;
				}
				UART1_OutString("\r\n");
			}
#endif

			if (data.runnumber == 0) {
				switch (turn_direction) {
					case left:			show_arrow(40, 4, left); 		break;
					case straight:	    show_arrow(56, 4, straight);	break;
					case right:			show_arrow(72, 4, right); 		break;
					case back:			show_arrow(56, 4, back);		break;
				}
			}

//			if (turn_direction != straight) {
//	            speed = data.turnspeed;
//	        }
//            Motor_Speed(speed, speed);
			
			while ((CURRENT_DISTANCE - start_position) < data.sensor_offset)  {
			    if (photo_data_ready) {
			        photo_data_ready = 0;
			        data_log(where_am_i << 8 | current_sensor, 1);
		            if (turn_direction != straight) {
                        speed -= data.acceleration;
                        if (speed < data.turnspeed) speed = data.turnspeed;
                        Motor_Speed(speed, speed);
		            }
			    }
			}

			if (turn(turn_direction)) {
				squareXY(0,0, 127, 63, 1);
				update_display();
				putstr(3, 3, "I AM LOST", 1);
				Motor_Stop();
				where_am_i = old_ret;
				return 1;
			}

#ifdef COLOR_SENSOR_ON_BACK
            switch (check_color()) {
                case red:
#ifdef WITH_BGX
                    if (STREAM_BIT) {
                        UART1_OutString("Dead end color: Red\r\n");
                    }
#endif
                    found_red = current_index;
                    LaunchPad_Output(RED);
                    if (explore_mode) break;
                    map[last_index].node_link[move_direction & TURN_MASK] = current_index;
                    goto finish;

                // if green we found start position - restart
                // if green field was start - we never will be here again!
                case green:
#ifdef WITH_BGX
                    if (STREAM_BIT) {
                        UART1_OutString("Dead end color: Green\r\n");
                    }
#endif
                    LaunchPad_Output(GREEN);
                    if (explore_mode) {
                        data.green_cell_nr = current_index;
                        data.pathlength = 0;
                    } else {
                        play = 0;
                        step = 0;
                        data.pathlength = 0;
                        move_direction = north;
                        my_coordinate.east = my_coordinate.north = 0;
                        goto full_restart;
                    }
                // Simply dead end - turn back.
                default:
#ifdef WITH_BGX
                    if (STREAM_BIT) {
                        UART1_OutString("Dead end color: None\r\n");
                    }
#endif
                    LaunchPad_Output(0);
                    break;
            }

#endif

			move_direction += turn_direction;
			move_direction &= TURN_MASK;

			
			if (map[current_index].pass_count[move_direction] < 2) {
				map[current_index].pass_count[move_direction] += 1;
			} else LaunchPad_Output(RED); // Ошибка!!! Выбран красный коридор!
			expect_index = map[current_index].node_link[move_direction];

			last_index = current_index;

			if (!play && !found_red) {
            // Store the intersection in the path variable.
				path[data.pathlength] = turn_direction;
				length[data.pathlength] = segment_length;  // до поворота
				data.pathlength++;
            // You should check to make sure that the path_length does not
            // exceed the bounds of the array.
				if (data.pathlength >= MAX_PATH_LENGTH) {
					squareXY(0,0, 127, 63, 1);
					update_display();
					putstr(0, 3, "MAX PATH LENGTH", 1);
					Motor_Speed(0, 0);
					speed  = 0;
					where_am_i = old_ret;
					return 1;
				}

        // Simplify the learned path.
				simplifyPath();

			} else {

				if (step >= data.pathlength) play = 0;

			}

    } while (1);

finish:
		// Проезжаем вперед на половину корпуса.
        if (play) {
            segment_length= CURRENT_DISTANCE  + 100 + data.sensor_offset;
            Motor_Speed(data.turnspeed, data.turnspeed);
            while (CURRENT_DISTANCE < segment_length) {
                if (CollisionFlag) {
                    CollisionFlag = 0;
                    break;
                }
            }
        }
#ifdef COLOR_SENSOR_ON_BACK
        else {
            segment_length= CURRENT_DISTANCE  - 140; // - data.sensor_offset;
            Motor_Speed(-data.turnspeed, -data.turnspeed);
            while (CURRENT_DISTANCE > segment_length) {
                if (CollisionFlag) {
                    CollisionFlag = 0;
                    break;
                }
            }
        }
#endif

save_map:
    Motor_Stop();
    BumpInt_Stop();
    where_am_i = old_ret;
    copy_data_dma((uint8_t *)length, (uint8_t *)data.length, data.pathlength*4);
    copy_data_dma(path, data.path, data.pathlength);
//    for (i=0; i<data.pathlength; i++ ){
//            data.path[i]   = (int) path[i];
//          data.length[i] =         length[i];
//    }
    data.red_cell_nr = found_red;

    if (explore_mode) {
        data.map_size = max_map_cell;
//			data_ptr = (uint32_t*) &map;
//			Flash_Erase(0x30000);
//			Flash_WriteArray(data_ptr, ROM_map_addr, sizeof(map)/4);
        spi_write_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));

    }
	return 0;
}

void BrakeTest(void) {
	unsigned int    photo_sensor, segment_length, length;
	CollisionFlag = 0;
	BumpInt_Init(&CollisionHandler);
	Motor_Enable();
	Motor_Speed(data.maxspeed, data.maxspeed);
    photo_sensor = 0;
    do {
		if (CollisionFlag) break;
        if (photo_data_ready ) {
            photo_data_ready  = 0;
            photo_sensor = current_sensor;
        }
    } while (photo_sensor == 0);
		if (!CollisionFlag) run_segment(slow, 0);
		BumpInt_Stop();
		segment_length = CURRENT_DISTANCE;
		Motor_Speed(0, 0);
		while (kbdread() != KEY_DOWN) {
			length = CURRENT_DISTANCE - segment_length;
			show_number(length, 0);
//            Clock_Delay1ms(100);
			delay_us(200*500);
		}
		Motor_Disable();
}

void FreeRun(void) {
	unsigned int travel_length;
	int stop_difference, turn_angle;

	CollisionFlag = 0;
	BumpInt_Init(&CollisionHandler);
	time = 0; speed = 0;
	Motor_Enable();
//	Motor_Speed(data.maxspeed, data.maxspeed); // 50%
  while(kbdread() != KEY_DOWN && time < data.timetorun*500) {

		if (photo_data_ready) {
			photo_data_ready = 0;
//				if (speed < data.maxspeed) speed += data.acceleration;
			if ((speed += data.acceleration) > data.maxspeed) speed = data.maxspeed;
			if (speed < 1500) speed = 1500;
			Motor_Speed(speed, speed);
		}			
		
    if (CollisionFlag) {

			speed = 0;
			travel_length = CURRENT_DISTANCE  - 100;
			Motor_Speed(-data.turnspeed, -data.turnspeed);
			while (CURRENT_DISTANCE > travel_length) continue;

			turn_angle = Bump_angle(CollisionData);
//			show_number(ABS(turn_angle), 0);
			if (turn_angle < 0) {
				turn_angle += 90;
				stop_difference = (RightSteps - LeftSteps) + DEGREE(turn_angle);
				Motor_Speed(-data.turnspeed, data.turnspeed);
				while ((RightSteps - LeftSteps) < stop_difference) continue;
			} else {
				turn_angle -= 90;
				stop_difference = (LeftSteps - RightSteps ) - DEGREE(turn_angle);
				Motor_Speed(data.turnspeed, -data.turnspeed);
				while ((LeftSteps - RightSteps) < stop_difference) continue;
			}

//			Clock_Delay1ms(400);
			CollisionData = 0;
			CollisionFlag = 0;
    }
  }
	Motor_Stop();
	BumpInt_Stop();
	Motor_Disable();
}

//unsigned int Remove_DeadEnds(void) {
//	unsigned int i, j, count, link, direction, removed = 0;
//
//	for (i = 1; i < data.map_size; i++) {
//		if (i != data.red_cell_nr) {
//			count = 0;
//			for (j = 0; j < 4; j++) {
//				if (map[i].node_link[j] == UNKNOWN) count++;
//				else direction = j, link = map[i].node_link[j];
//			}
//			if (count == 3) {
//				map[i].node_link[direction] = UNKNOWN;
//				direction += back;
//				direction &= TURN_MASK;
//				map[link].node_link[direction] = UNKNOWN;
//				removed++;
//			}
//		}
//	}
//	return removed;
//}



unsigned int calculation_time;
static unsigned int path_length[MAX_MAP_SIZE*2];

#define QUEUE_SIZE  64
static unsigned int sort_idx[QUEUE_SIZE], sort_tail=0;

static unsigned int insert_val(unsigned int val, unsigned int idx) {
    unsigned int i;

    if (++sort_tail > (QUEUE_SIZE-1)) return 1;
    for (i = sort_tail-1; i > 0; i--) {
        if(path_length[sort_idx[i-1]] < val) {
            sort_idx[i] = sort_idx[i-1];
        } else break;
    }
    sort_idx[i] = idx;
    return 0;
}

static unsigned int extract_val(void) {
    return sort_idx[--sort_tail];
}

//#define SLICED_MAP
void Search_Short_Way_with_turns(void) {

    unsigned int k, min_index, destination;
#ifdef SLICED_MAP
    unsigned int i, j;
	static two_layer_map_cell_t map2d [MAX_MAP_SIZE * 2];
#endif
	int	distance;
	int prev_bearing, bearing;

	fill_data32_dma(0xffffffff, path_length, data.map_size*2);
	data.pathlength = 0;

	spi_read_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));

	// переводим в двухслойную карту
	// однозначное соответствие: каждый узел делится на два n*2 и (n*2)+1
	// между ними сегмент длиной стоимостью поворота. К четным узлам примыкают
	// меридианальные сегменты - север/юг, к нечетным широтные - запад/восток.
	// каждый из этих сегментов удлиняется на стоимость прямого прохода узла.//
    benchmark_start();
#ifdef SLICED_MAP
    for (i = 0; i < data.map_size; i++) {
		map2d[i*2].link[2] = i*2+1;
		map2d[i*2].length[2] = data.turncost;
		map2d[i*2+1].link[2] = i*2;
		map2d[i*2+1].length[2] = data.turncost;
		for (j = north; j <= west; j++) {
			if ((destination = map[i].node_link[j]) == UNKNOWN) {
				map2d[i*2 + (j&0x01)].link[(j>>1)] = UNKNOWN;
			} else {
				map2d[i*2 + (j&0x01)].link[(j>>1)] = (destination * 2 + (j & 0x01));			
				map2d[destination * 2 + (j & 0x01)].link[(j>>1) ^ 0x01] = i*2 + (j & 0x01);
				distance = (map[i].coordinate.north - map[destination].coordinate.north) +
									 (map[i].coordinate.east  - map[destination].coordinate.east);
				if (distance < 0) distance = -distance;
				map2d[i*2 + (j&0x01)].length[(j>>1)] = distance + (data.crosscost);
				map2d[destination * 2 + (j & 0x01)].length[(j>>1) ^ 0x01] = distance + (data.crosscost);
			}
		}
	}
#endif
	while (dma_copy_busy) continue; // ждём окончания fill_dma path_length[] <= 0xFFFFFFFF
    // Начинаем поиск с конца - стартовая точка финиш.
    // Причем, прибытие с любой стороны допустимо.
    path_length[data.red_cell_nr * 2] = 0;
    path_length[data.red_cell_nr * 2 + 1] = 0;

    sort_tail = 0;
    insert_val(0, data.red_cell_nr * 2);
    insert_val(0, data.red_cell_nr * 2 + 1);

#ifdef SLICED_MAP
    // Составляем карту высот по алгоритму Дейкстры
    while (sort_tail) {
        min_index  = extract_val();
        for (k = 0; k < 3; k++) {
            if ((i = map2d[min_index].link[k]) != UNKNOWN) {
                distance = map2d[min_index].length[k];
                if (path_length[i] > (path_length[min_index] + distance)) {
                    path_length[i] = path_length[min_index] + distance;
                    insert_val(path_length[i], i);
                }
            }
        }
    }
#else
    // Составляем карту высот по алгоритму Дейкстры
    while (sort_tail) {
        min_index  = extract_val();
        for (k = 0; k < 2; k++) {
            if ((destination = map[min_index/2].node_link[2*k + (min_index & 0x01)]) != UNKNOWN) {
//                distance = map2d[min_index].length[k];
                distance =   (map[min_index/2].coordinate.north - map[destination].coordinate.north) +
                             (map[min_index/2].coordinate.east  - map[destination].coordinate.east);
                if (distance < 0) distance = -distance + data.crosscost;
                else              distance += data.crosscost;
                destination = destination * 2 + (min_index & 0x01);
                if (path_length[destination] > (path_length[min_index] + distance)) {
                    path_length[destination] = path_length[min_index] + distance;
                    insert_val(path_length[destination], destination);
                }
            }
        }
        distance = data.turncost;
        destination = min_index ^ 0x01;
        if (path_length[destination] > (path_length[min_index] + distance)) {
            path_length[destination] = path_length[min_index] + distance;
            insert_val(path_length[destination], destination);
        }
    }
#endif
	// Теперь скатываемся вниз для создания кратчайшего пути
	// от старта к финишу.
	data.pathlength = 0; // указатель пути на начало
	prev_bearing = north;  // не обязательно
	min_index = data.green_cell_nr << 1 ;  // начинаем с точки 0
	if (path_length[min_index] > path_length[min_index+1]) {
		min_index++;
		prev_bearing = east;
	}
#ifdef SLICED_MAP
	while ((min_index >> 1) != data.red_cell_nr) {
		for (k = 0; k < 3; k++	) {
			if ((destination = map2d[min_index].link[k]) != UNKNOWN) {
				distance = map2d[min_index].length[k];
				if ((path_length[min_index] - distance) == path_length[destination]) {
					if (k < 2) { // если выбрано направление из узла
					            //  k=2 - смена широтное/меридианальное направление и в путь не записывется
						bearing = (bearing_dir_t)((min_index & 0x01) | (k << 1));
						path[data.pathlength] = (rotation_dir_t)(bearing - prev_bearing);
						path[data.pathlength] &= TURN_MASK;
						prev_bearing = bearing;
						if ((min_index >> 1) != data.green_cell_nr) {
						    data.pathlength++;
	                        length[data.pathlength] = distance-data.crosscost;  // тут проблема возможно!!!!!!!!!!!!!
						} else {
	                        length[data.pathlength] = distance-data.crosscost;  // тут проблема возможно!!!!!!!!!!!!!
						}
					}
					min_index = destination;
					break;
				}
			}
		}
	}
#else
	while (path_length[min_index]) {
	    for (k = 0; k < 3; k++) {
	        if (k < 2) {
                if ((destination = map[min_index/2].node_link[2*k + (min_index & 0x01)]) != UNKNOWN) {
                    distance =   (map[min_index/2].coordinate.north - map[destination].coordinate.north) +
                                 (map[min_index/2].coordinate.east  - map[destination].coordinate.east);
                    if (distance < 0) distance = -distance + data.crosscost;
                    else              distance += data.crosscost;
                    destination = destination*2 + (min_index & 0x01);
                    if ((path_length[min_index] - distance) == (path_length[destination])) {
                        bearing = (bearing_dir_t)((min_index & 0x01) + 2*k);
                        path[data.pathlength] = (rotation_dir_t)(bearing - prev_bearing);
                        path[data.pathlength] &= TURN_MASK;
                        prev_bearing = bearing;
                        if ((min_index >> 1) != data.green_cell_nr) {
                            data.pathlength++;
                            length[data.pathlength] = distance-data.crosscost;  // тут проблема возможно!!!!!!!!!!!!!
                        } else {
                            length[data.pathlength] = distance-data.crosscost;  // тут проблема возможно!!!!!!!!!!!!!
                        }
                        min_index = destination;
                        break;
                    }
                }
	        } else {
	            if ((path_length[min_index] - data.turncost) == (path_length[min_index ^ 0x01])) {
	                min_index = min_index ^ 0x01;
	                break;
	            }
            }
	    }
	}
#endif
	path[data.pathlength++] = back;//
    calculation_time = benchmark_stop();
//	for (i=0; i< data.pathlength; i++) {
//		data.path[i] = path[i];
//	}
	copy_data_dma(path, data.path, data.pathlength);
	copy_data_dma((uint8_t *) length, (uint8_t *)data.length, (data.pathlength) * sizeof(length[0]));
}

//#ifdef PLAY
unsigned int PlayMaze(void) {
    unsigned int step=0;
    rotation_dir_t   turn_direction = straight;
    int segment_length;

    where_am_i = Solve;
    data_log_init();

    if (data.pathlength == 0) return 1;
    copy_data_dma(data.path, path, data.pathlength);
    copy_data_dma((uint8_t *)data.length, (uint8_t *)length, data.pathlength * 4);

    CollisionFlag = 0;
    BumpInt_Init(&CollisionHandler);
    LaunchPad_Output(GREEN);

// Проезжаем вперед на половину корпуса вслепую
    segment_length= CURRENT_DISTANCE  + 100;// - data.sensor_offset;
    speed = 0;
    while (CURRENT_DISTANCE < segment_length) {
        if (photo_data_ready) {
            photo_data_ready = 0;
            data_log(where_am_i << 8 | current_sensor, 1);
            if ((speed += data.acceleration) > data.maxspeed) speed = data.maxspeed;
            Motor_Speed(speed, speed);
        }
    }


    do {

        LaunchPad_Output(0);
        // Navigate current line segment

        if (path[step] == straight) {
            run_segment(fast, length[step]);
        } else {
            if (length[step] > data.brake_path) {
                run_segment(fast, length[step] - data.brake_path);
            } else {
                run_segment(slow, 0);
            }
        }

        turn_direction = path[step++];
        if (turn_direction == back) break;


        if (turn(turn_direction)) {
            squareXY(0,0, 127, 63, 1);
            update_display();
            putstr(3, 3, "I AM LOST", 1);
            Motor_Stop();
            where_am_i = old_ret;
            return 1;
        }
    } while(1);

    // Проезжаем вперед на половину корпуса.
    segment_length= CURRENT_DISTANCE  + 100 + data.sensor_offset;
    while (CURRENT_DISTANCE < segment_length) {
        if (CollisionFlag) {
            CollisionFlag = 0;
            break;
        }
        if (photo_data_ready) {
            photo_data_ready = 0;
            data_log(where_am_i << 8 | current_sensor, 1);
            if ((speed -= data.acceleration) < data.turnspeed) speed = data.turnspeed;
            Motor_Speed(speed, speed);
        }
    }
    Motor_Stop();
    BumpInt_Stop();
    where_am_i = old_ret;

    return 0;
}

#include "square_maze.h"

bearing_dir_t real_bearing;
unsigned int my_position;
coordinate_t real_coordinate;

void init_virtual_maze(void) {
    coordinate_t start = {12, 5};
    spi_read_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));
    init_maze(15, 7, start, north);
//    draw_maze(start);

    my_position = data.green_cell_nr;
    real_coordinate = map[my_position].coordinate;
    real_bearing = north;
}
uint8_t get_walls(void) {
    unsigned int ii, bb = real_bearing,
    result = 0, mask = 0x01;

    for (ii = 0; ii < 4; ii++, mask <<= 1) {
        result |= (map[my_position].node_link[bb] == UNKNOWN) ? mask : 0;
        bb++;
        bb &= TURN_MASK;
    }
    return result;
}

unsigned int make_step(void) {
    int previous_coordinate, distance;

    if (map[my_position].node_link[real_bearing] == UNKNOWN) return 0;

    switch (real_bearing) {
        case north:
            previous_coordinate = map[my_position].coordinate.north;
            my_position = map[my_position].node_link[real_bearing];
            distance = map[my_position].coordinate.north - previous_coordinate;
            break;
        case south:
            previous_coordinate = map[my_position].coordinate.north;
            my_position = map[my_position].node_link[real_bearing];
            distance = previous_coordinate - map[my_position].coordinate.north;
            break;
        case east:
            previous_coordinate = map[my_position].coordinate.east;
            my_position = map[my_position].node_link[real_bearing];
            distance = map[my_position].coordinate.east - previous_coordinate;
            break;
        case west:
            previous_coordinate = map[my_position].coordinate.east;
            my_position = map[my_position].node_link[real_bearing];
            distance = previous_coordinate - map[my_position].coordinate.east;
            break;
    }
    return (distance + (cell_step/2))/cell_step;;
}

bearing_dir_t make_turn( turn_direction) {
    real_bearing += turn_direction;
    return (bearing_dir_t) (real_bearing &= TURN_MASK);
}

t_color get_color(void) {
    if (my_position == data.green_cell_nr) return green;
    if (my_position == data.red_cell_nr)    return red;
    return white;
}
