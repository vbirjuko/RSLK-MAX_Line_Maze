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
#include "FRAM_Logging.h"
#include "UART0.h"

#ifdef WITH_BGX
	#include "UART1.h"
#endif


#define MAX_PATH_LENGTH (MAX_MAP_SIZE * 2)
rotation_dir_t path[MAX_PATH_LENGTH];
int length[MAX_PATH_LENGTH];

// 220mm per 360 tick of two wheels.
//#define CURRENT_DISTANCE	((LeftSteps + RightSteps) * 11 / 36)
#define CURRENT_DISTANCE    STEPS_TO_MM(LeftSteps + RightSteps)
#define MIN_RPM (733333333/65536/2)

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
	fast,
	slow_with_break,
	fast_with_break,
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
// если задана быстрая скорость, то робот движется со скоростью maxspeed на участке distance, затем сбавляет до turnspeed.
// если задана медленная скорость, то весь сегмент движется со скоростью minspeed.
// в пределах guarddist игнорируется пропадание линии и срабатывание боковых сенсоров.
void run_segment(speed_t runspeed, unsigned int distance) {
    unsigned int activ_sensor_count, last_sensor, first_sensor, photo_mask;
    unsigned int blind, blindcount = 0, i, finishcount = 0, slowdistance, ignoredistance; 
    int track_error, de_dt, sigma_error = 0, prev_track_error=0, left_speed, right_speed, result, maxspeed;
	uint8_t backlight = 0;

    old_ret = where_am_i;
    if (where_am_i) where_am_i = Segment;

    if (runspeed == slow || runspeed == slow_with_break) maxspeed = data.minspeed, backlight = BK_LEFT | BK_RGHT;
    else				                                 maxspeed = data.maxspeed, backlight = FR_LEFT | FR_RGHT;

    if (runspeed == fast_with_break || runspeed == slow_with_break) {
        int brakepath1, brakepath2;
        if (maxspeed >=  data.minspeed) {
            // Расстояние, необходимое для торможения, если максимальная скорость не набирается:
            //  (1mm/100)^2 * (V^2 - v^2) / (4 * a * 400fps/100)  + distance/2
            brakepath1 = (long long)(speed + data.minspeed)*(speed - data.minspeed)/ data.acceleration / 160000 + distance/2;
            // Расстояние, необходимое для торможения от максимальной скорости:
            // (1mm/100)^2 * (V^2 - v^2) / (2 * a * 400fps/100)
            brakepath2 = (long long)(maxspeed + data.minspeed)*(maxspeed - data.minspeed) / data.acceleration / 80000;
            // используем вариант с самым коротким тормозным путём
            if ((brakepath1 < brakepath2) && (brakepath1 >= 0)) {
                slowdistance = CURRENT_DISTANCE + distance - data.sensor_offset - brakepath1;
            } else {
                slowdistance = CURRENT_DISTANCE + distance - data.sensor_offset - brakepath2;
            }

        } else slowdistance = CURRENT_DISTANCE + distance;
    } else     slowdistance = CURRENT_DISTANCE + distance;

#ifdef BLINKER_SEGMENT
    Blinker_Output(backlight);
#else
    (void)backlight;
#endif
    ignoredistance = CURRENT_DISTANCE + data.guarddist;
	
    while(1) {
        if (photo_data_ready ) {
            if (CollisionFlag) return;
            photo_data_ready  = 0;
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif
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
//// Для ПИД коррекции сужаем аппертуру сенсоров, участвующих в вычислении ошибки позиции
//// так как к этому времени робот уже должен быть стабилизирован на линии. - нет, что-то не то.
//                    if (first_sensor < 2) first_sensor = 2;
//                    if (last_sensor  > 5) last_sensor  = 5;
                }
                track_error = line_error(first_sensor, last_sensor);
            }
            
// Теперь имеем значение ошибки/смещения и дифференциальное значение ошибки

            de_dt = data.k_diff*(track_error - prev_track_error);
            prev_track_error = track_error;

            sigma_error += track_error;
            if (sigma_error > 8192) sigma_error = 8192;
            if (sigma_error < -8192) sigma_error = -8192;

            result = data.k_error*track_error + de_dt + ((sigma_error * data.k_integral) >> 10);

//            if (result < -data.maxmotor) result = -data.maxmotor;
//            else if (result > data.maxmotor) result = data.maxmotor;

            if (CURRENT_DISTANCE >= slowdistance) {
                if (maxspeed > data.minspeed) maxspeed -= data.acceleration;
                else 	maxspeed = data.minspeed;
                backlight = BK_LEFT | BK_RGHT;
#ifdef BLINKER_SEGMENT
                Blinker_Output(backlight);
#endif
            }
//                if ((speed + (ABS(result)*speed)/8192) > data.maxspeed) {
//                    speed = data.maxspeed - (ABS(result)*speed)/8192;
//                    if (speed < data.minspeed) speed = data.minspeed;  // ??????????????
//                }

            if ((ABS(result)) < (data.on_way*data.k_error)) {
                if ((speed += data.acceleration) > maxspeed) speed = maxspeed;
                if (speed < MIN_RPM) speed = MIN_RPM;
            }
            left_speed = speed+(result*speed)/8192; // speed*(1+result/nom_speed)
            right_speed = speed-(result*speed)/8192;

//            if (left_speed > data.maxmotor) left_speed = data.maxmotor;
//            else if (left_speed < -data.maxmotor) left_speed = -data.maxmotor;
//
//            if (right_speed > data.maxmotor) right_speed = data.maxmotor;
//            else if (right_speed < -data.maxmotor) right_speed = -data.maxmotor;

            Motor_Speed(left_speed, right_speed);
        }
    }
}

// 143mm wide ~ 449 mm length / 220mm wheel step and multiply 360 pulse = 735 pulse
// 180grad ~ 735
// x*735/180   /5
// x*147/36    /3
// x*49/12
#define DEGREE(x)  ((x*49)/12)

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

#define DEGREE_TO_STEPS(X) ((TRACK_WIDE*314L*36L*(X)+(11L*360*100/2))/(11L*360*100))

    stop_difference = MM_TO_STEPS((degree == 180) ? (TRACK_WIDE * 135 * 314 / 100 / 360) : (TRACK_WIDE *  45 * 314 / 100 / 360));
    fail_difference = MM_TO_STEPS((degree == 180) ? (TRACK_WIDE * 360 * 314 / 100 / 360) : (TRACK_WIDE * 135 * 314 / 100 / 360));
//    slow_difference = ((degree == 180) ? DEGREE_TO_STEPS(120) : DEGREE_TO_STEPS(60));
    {
//        int circle_length = degree * TRACK_WIDE * 314 / 100 / 360;
        int circle_length = (degree == 180) ? (180 * TRACK_WIDE * 314)+(100*360/2) / (100 * 360) : (90 * TRACK_WIDE * 314)+(100*360/2) / (100 * 360);
        if ((brakepath * 2) > circle_length) {
            slow_difference = MM_TO_STEPS(circle_length/2);
        } else {
            slow_difference = MM_TO_STEPS(brakepath);
        }
    }

    switch (turn_dir) {
        case left:      // turn left
            speed = 0;
            last_turn = left;
            stop_difference += (RightSteps - LeftSteps);
            slow_difference += (RightSteps - LeftSteps);
            fail_difference += (RightSteps - LeftSteps);
            do {
                count = 0;
                while(count < 2) {
                    if (photo_data_ready ) {
                        photo_data_ready  = 0;
                        photo_sensor = current_sensor;
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif

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
            stop_difference += (LeftSteps - RightSteps);
            slow_difference += (LeftSteps - RightSteps);
            fail_difference += (LeftSteps - RightSteps);
            do {
                count = 0;
                while(count < 2) {
                    if (photo_data_ready ) {
                        photo_data_ready  = 0;
                        photo_sensor = current_sensor;
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif

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
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif
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
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif
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
rotation_dir_t SelectTurn(unsigned int available_directions, const unsigned int * sequence) {
    unsigned int  i;
    if (available_directions) {
        for (i = 0; i < 3; i++) {
            if (available_directions & sequence[i]) break;
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
	unsigned int maze_entrance = 1, available, photo_sensor, step=0, play = 0, ii, blind;
	unsigned int skiplevel, maxspeed;
	unsigned int last_index = 0, current_index, count_green, count_yellow, count_red, found_red = 0;
	unsigned int expect_index = UNKNOWN, turn_index;
    volatile unsigned int record_count = 0;
	rotation_dir_t   turn_direction = straight;
	int		move_direction = north;
	coordinate_t my_coordinate = {0, 0};
	unsigned int max_map_cell = 0;
	int segment_length, start_position;

	where_am_i = Entrance;
	InitBrakePath();

#if FRAM_SIZE == 0
	data_log_init();
#else
    FRAM_dma_log_Start(0x0004);
    frames_to_go = FRAM_SIZE/sizeof(data_buffer_t);
#endif
	CollisionFlag = 0;
	BumpInt_Init(&CollisionHandler);
	LaunchPad_Output(GREEN);
	if (explore_mode)   maxspeed = data.minspeed;
	else                maxspeed = data.maxspeed;
	
// Проезжаем вперед на половину корпуса вслепую
	segment_length= CURRENT_DISTANCE  + 100;// - data.sensor_offset;
	speed = 0;
	while (CURRENT_DISTANCE < segment_length) {
	    if (photo_data_ready) {
	        photo_data_ready = 0;
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif
            if (current_sensor) break;
	        if ((speed += data.acceleration) > maxspeed) speed = maxspeed;
	        if (speed < 1500) speed = 1500;
	        Motor_Speed(speed, speed);
	    }
	}
	segment_length = 0;

full_restart:
    where_am_i = Solve;
    if (explore_mode) {
        max_map_cell = 0;
        found_red = 0;
        last_index = 0;
        maze_entrance = 1;
        data.pathlength = 0;
        data.green_cell_nr = 0;
        play = 0;

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
    } else {
        if (data.pathlength > 0) play = 1;
        copy_data_dma(data.path, path, data.pathlength);
        copy_data_dma((uint8_t *)data.length, (uint8_t *)length, data.pathlength * 4);

        if ((data.green_cell_nr > data.map_size) || (data.red_cell_nr > data.map_size)) data.map_size = 0;
        max_map_cell = data.map_size;
        for (ii = 0; ii < max_map_cell; ii++) {
            if (map[ii].pass_count[0] < 4) map[ii].pass_count[0] = 0;
            if (map[ii].pass_count[1] < 4) map[ii].pass_count[1] = 0;
            if (map[ii].pass_count[2] < 4) map[ii].pass_count[2] = 0;
            if (map[ii].pass_count[3] < 4) map[ii].pass_count[3] = 0;
        }
        if (max_map_cell) {
            last_index = data.green_cell_nr;
            my_coordinate = map[data.green_cell_nr].coordinate;
            for (ii = 0; ii < 4; ii++) {
                if (map[data.green_cell_nr].pass_count[ii] == 0) {
                    map[data.green_cell_nr].pass_count[ii] = 1;
                    move_direction = ii;
                    break;
                }
            }
        } else { // если карта совсем пустая
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
            maze_entrance = 1;

        }
    }

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

		// если выполняем маршрут, то сегмент, после которого не надо делать поворот, проходим
		// на максимальной скорости, если будет поворот, то часть проходим на максимуме, а за
		// brake path до конца начинаем тормозить.
		if (play) {
			if (path[step] == straight) {
				run_segment(fast, length[step]);
			} else {
			    run_segment(fast_with_break, length[step]);
//				if ((length[step]/2) > data.brake_path) {
//					run_segment(fast, length[step] - data.brake_path);
//				} else {
//					run_segment(fast, length[step]/2);
//				}
			}
		} else {
			run_segment(slow, 0);
//			run_segment(fast_with_break, segment_length);
		};

		if (CollisionFlag) {
		    CollisionFlag = 0;
			squareXY(0,0, 127, 63, 1);
			update_display();
			putstr(3, 3, "BUMPER HIT", 1);
			where_am_i = old_ret;
#if FRAM_SIZE == 0
			data_log_finish();
#else
			if (FRAM_dma_log_Stop()) LaunchPad_Output(RED);
		    record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
		    if (FRAM_dma_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
		    if (FRAM_dma_log_write((uint8_t*)&record_count, sizeof(record_count))) LaunchPad_Output(RED|BLUE);
		    if (FRAM_dma_log_Stop()) LaunchPad_Output(BLUE);
#endif
		    return 1;
		}
		LaunchPad_Output(BLUE);
		maze_entrance = 0;
		segment_length = CURRENT_DISTANCE - start_position;
		start_position = CURRENT_DISTANCE;

        // если указана величина клетки, то подгоняем длину сегмента под этот шаг.
        if (data.cell_step) {
          segment_length = ((segment_length + data.cell_step/2) / data.cell_step) * data.cell_step;
          if (segment_length == 0) segment_length = data.cell_step;
        }

		update_coordinate(&my_coordinate, segment_length, (bearing_dir_t)move_direction);

		available = 0;
		photo_sensor = current_sensor;

		// Check for left and right exits.
		if(photo_sensor & (1u << 7)) { available |= LEFT_MASK; }
		if(photo_sensor & (1u << 0)) { available |= RIGHT_MASK; }
		if (where_am_i)  where_am_i = Solve | available;

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
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif

				// проверка условия непрерывности линии
				for (ii=0; ii< 8; ii++) {
				    if (prev_stat != (BITBAND_SRAM(photo_sensor, ii))) {
		                if (prev_stat = BITBAND_SRAM(photo_sensor, ii) == 1) linecount++;
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
#if FRAM_SIZE == 0
            data_log_finish();
#else
            if (FRAM_dma_log_Stop()) LaunchPad_Output(RED);
            record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
            if (FRAM_dma_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
            if (FRAM_dma_log_write((uint8_t*)&record_count, sizeof(record_count))) LaunchPad_Output(RED|BLUE);
            if (FRAM_dma_log_Stop()) LaunchPad_Output(BLUE);
#endif
					return 1;
				}
			}
		}

//		if (available != (LEFT_MASK | RIGHT_MASK)) { // если поворот, то делаем небольшую задержку
		if (1) {
		ii=10;
			blind = 0;
			while (ii) {
//		    while ((CURRENT_DISTANCE - start_position) < LINE_WIDTH) {
				if (photo_data_ready ) {
					photo_data_ready  = 0;
					photo_sensor = current_sensor;
					if (photo_sensor == 0) blind = 1;
#if FRAM_SIZE == 0
                    data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif
					ii--;
				}
			}
		}

		// now detect if we have straight segment
		if ((photo_sensor & ((1u << 2) | (1u << 3) | (1u << 4) | (1u << 5))) && !blind) {
			available |= STRAIGHT_MASK; 
			if (where_am_i)  where_am_i = Solve | available;
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
#if FRAM_SIZE == 0
            data_log( (my_coordinate.east & 0x000000ff)			| (coord_east_lsb << 8), 1);
            data_log( ((my_coordinate.east & 0x0000ff00) >> 8)	| (coord_east_msb << 8), 1);
            data_log( (my_coordinate.north & 0x000000ff) 		| (coord_north_lsb << 8), 1);
            data_log( ((my_coordinate.north & 0x0000ff00)>> 8)	| (coord_north_msb << 8), 1);
            data_log( (segment_length & 0x000000ff) 			| (segm_length_lsb << 8), 1);
            data_log( ((segment_length & 0x0000ff00)>> 8)		| (segm_length_msb << 8), 1);
            data_log( (current_index & 0xff)                    | (node_idx << 8) , 1);
#else
            FRAM_log_slow(my_coordinate, segment_length, current_index);
#endif
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
#if FRAM_SIZE == 0
            data_log_finish();
#else
            if (FRAM_dma_log_Stop()) LaunchPad_Output(RED);
            record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
            if (FRAM_dma_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
            if (FRAM_dma_log_write((uint8_t*)&record_count, sizeof(record_count))) LaunchPad_Output(RED|BLUE);
            if (FRAM_dma_log_Stop()) LaunchPad_Output(BLUE);
#endif
						return 1;
					}
				}
				if (map[current_index].pass_count[(move_direction + back) & TURN_MASK] < 2) {
						map[current_index].pass_count[(move_direction + back) & TURN_MASK] += 1;
				}
				map[current_index].node_link [(move_direction + back) & TURN_MASK] = last_index;

				count_green = 0; count_yellow = 0; count_red = 0;
				for (ii = 0; ii < 4; ii++) {
					switch (map[current_index].pass_count[ii]) {
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

			turn_index = ((unsigned int)data.lefthand < 6u) ? data.lefthand : Rand() % 6;
				
			if (play) {
			    turn_direction = path[step++];
			    if (turn_direction == back) break;
			}
			else	turn_direction = SelectTurn(available, turn_sequence[turn_index]);

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
#if FRAM_SIZE == 0
			        data_log(where_am_i << 8 | current_sensor, 1);
#else
                    FRAM_log_data();
#endif
		            if (turn_direction != straight) {
                        if ((speed -= data.acceleration) < data.turnspeed) speed = data.turnspeed;
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
#if FRAM_SIZE == 0
				data_log_finish();
#else
                if (FRAM_dma_log_Stop()) LaunchPad_Output(RED);
                record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
                if (FRAM_dma_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
                if (FRAM_dma_log_write((uint8_t*)&record_count, sizeof(record_count))) LaunchPad_Output(RED|BLUE);
                if (FRAM_dma_log_Stop()) LaunchPad_Output(BLUE);
#endif
				return 1;
			}

#ifdef COLOR_SENSOR_ON_BACK
			if (turn_direction == back) {
			    Motor_Speed(0, 0);
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
                        path[data.pathlength] = back;
                        length[data.pathlength] = segment_length;
                        data.pathlength++;
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
#if FRAM_SIZE == 0
					data_log_finish();
#else
                    if (FRAM_dma_log_Stop()) LaunchPad_Output(RED);
                    record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
                    if (FRAM_dma_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
                    if (FRAM_dma_log_write((uint8_t*)&record_count, sizeof(record_count))) LaunchPad_Output(RED|BLUE);
                    if (FRAM_dma_log_Stop()) LaunchPad_Output(BLUE);
#endif
					return 1;
				}

        // Simplify the learned path.
				simplifyPath();

			} else {
				if (step >= data.pathlength) play = 0;
			}
            if (expect_index != UNKNOWN) { // то надо дать знать длину сегмента.
                switch (move_direction) {
                case north:
                    segment_length = map[expect_index].coordinate.north - map[current_index].coordinate.north;
                    break;
                case south:
                    segment_length = map[current_index].coordinate.north - map[expect_index].coordinate.north;
                    break;
                case east:
                    segment_length = map[expect_index].coordinate.east - map[current_index].coordinate.east;
                    break;
                case west:
                    segment_length = map[current_index].coordinate.east - map[expect_index].coordinate.east;
                    break;
                }
            } else segment_length = 0;
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
                if (photo_data_ready) {
                    photo_data_ready = 0;
                    FRAM_log_data();
                    if ((speed -= data.acceleration) < data.turnspeed) speed = data.turnspeed;
                    Motor_Speed(speed, speed);
                }
            }
#ifdef COLOR_SENSOR_ON_BACK
        } else {
            segment_length= CURRENT_DISTANCE  - 140; // - data.sensor_offset;
            speed = 0;
            Motor_Speed(speed, speed);
            while (CURRENT_DISTANCE > segment_length) {
                if (CollisionFlag) {
                    CollisionFlag = 0;
                    break;
                }
                if (photo_data_ready) {
                    photo_data_ready = 0;
                    FRAM_log_data();
                    speed -= data.acceleration;
                    if (speed < -data.turnspeed) speed = -data.turnspeed;
                    Motor_Speed(speed, speed);
                }
            }
#endif
        }

save_map:
#if FRAM_SIZE == 0
            data_log_finish();
#else
            if (FRAM_dma_log_Stop()) LaunchPad_Output(RED);
            record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
            if (FRAM_dma_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
            if (FRAM_dma_log_write((uint8_t*)&record_count, sizeof(record_count))) LaunchPad_Output(RED|BLUE);
            if (FRAM_dma_log_Stop()) LaunchPad_Output(BLUE);
#endif
    Motor_Stop();
    BumpInt_Stop();
    where_am_i = old_ret;
    copy_data_dma((uint8_t *)length, (uint8_t *)data.length, data.pathlength*4);
    copy_data_dma(path, data.path, data.pathlength);
//    for (i=0; i<data.pathlength; i++ ){
//            data.path[i]   = (int) path[i];
//          data.length[i] =         length[i];
//    }
    if (found_red) data.red_cell_nr = found_red;

    if (explore_mode) {
        data.map_size = max_map_cell;
        spi_write_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));
    }
	return 0;
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


unsigned int calculation_time;
static unsigned int path_length_table[MAX_MAP_SIZE*2];

#define QUEUE_SIZE  64
static unsigned int sort_idx[QUEUE_SIZE], sort_tail=0;

static unsigned int insert_val(unsigned int val, unsigned int idx) {
    unsigned int i;

    if (++sort_tail > (QUEUE_SIZE-1)) return 1;
    for (i = sort_tail-1; i > 0; i--) {
        if(path_length_table[sort_idx[i-1]] < val) {
            sort_idx[i] = sort_idx[i-1];
        } else break;
    }
    sort_idx[i] = idx;
    return 0;
}

static unsigned int extract_val(void) {
    if (sort_tail)return sort_idx[--sort_tail];
    else          return sort_idx[sort_tail];
}

int brakepath = 0;
/*
* Функция вычисляет время необходимое для проезда заданного расстояния
* Расстояник задаётся в миллиметрах, результат в миллисекундах
*/
__attribute__ ((ramfunc)) int TimeToRunStraight(int distance) {  // in milliseconds from millimeters.
  int result = 0;
  if (distance < brakepath * 2) {
      // t = sqrt(4*S/a)
      // V = 0.01 mm * speed per second = 0.01mm /1000ms =
      // => speed / 100 000 mm/ms
      // a = 0.01 mm * 400 * data.acceleration = 4*data.acceleration mm/s^2 =
      // = 4*10^-6 * data.accel mm/ms^2
      result = sqrt(1000000LL*distance/data.acceleration);
  } else {
      // t = (S + Sbr) / Vmax
      result = 100000LL*(distance + 2*brakepath)/(data.maxmotor);
  }
  if (result) return result;
  else return 1;
}

void InitBrakePath(void) {
  // Расстояние, необходимое для торможения от максимальной скорости:
  // (V^2 - v^2) / (2 * a)
    // V = 0.01 mm * speed
    // a = 0.01 mm * 400 * data.acceleration
  brakepath = (long long)data.maxmotor*data.maxmotor/data.acceleration/(10000*4*2);
  // turn length: 143mm wide * Pi / 4 = 112mm - each wheel run to turn 90 degree.
  data.turncost = TimeToRunStraight((TRACK_WIDE * 314 + 200)/400);
}

unsigned int Search_Short_Way_with_turns(void) {

    unsigned int ii, k, min_index, destination, destination_map, break_flag = 0;
    int   distance;
    int prev_bearing, bearing;

    fill_data32_dma(0xffffffff, (uint32_t*)path_length_table, data.map_size*2);
    data.pathlength = 0;

    InitBrakePath();
    spi_read_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));

    // переводим в двухслойную карту
    // однозначное соответствие: каждый узел делится на два n*2 и (n*2)+1
    // между ними сегмент длиной стоимостью поворота. К четным узлам примыкают
    // меридианальные сегменты - север/юг, к нечетным широтные - запад/восток.
    // каждый из этих сегментов удлинняется на стоимость прямого прохода узла.

    benchmark_start();
    while (dma_copy_busy) continue; // ждём окончания fill_dma path_length[] <= 0xFFFFFFFF
    // Начинаем поиск с конца - стартовая точка финиш.
    // Причем, прибытие с любой стороны допустимо.
    path_length_table[data.red_cell_nr * 2] = 0;
    path_length_table[data.red_cell_nr * 2 + 1] = 0;

    sort_tail = 0;
    if (insert_val(0, data.red_cell_nr * 2)) return 1;
    if (insert_val(0, data.red_cell_nr * 2 + 1)) return 1;

    // Составляем карту высот по алгоритму Дейкстры
    while (sort_tail) {
        min_index  = extract_val();
        for (k = 0; k < 2; k++) {
            unsigned int next_min_index = min_index;
            while ((destination_map = map[next_min_index/2].node_link[2*k + (next_min_index & 0x01)]) != UNKNOWN) {
                distance =  (min_index & 0x01) ? (map[min_index/2].coordinate.east  - map[destination_map].coordinate.east) :
                        (map[min_index/2].coordinate.north - map[destination_map].coordinate.north);
                if (distance < 0) distance = -distance;
                distance = TimeToRunStraight(distance); // переводим миллиметры в миллисекунды.

                destination = destination_map * 2 + (next_min_index & 0x01);
                if (path_length_table[destination] > (path_length_table[min_index] + distance)) {
                    path_length_table[destination] = path_length_table[min_index] + distance;
                    if (insert_val(path_length_table[destination], destination)) return 1;
                }
                next_min_index = map[next_min_index/2].node_link[2*k + (next_min_index & 0x01)]*2 + (min_index & 0x01);
            }
        }
        distance = data.turncost;
        destination = min_index ^ 0x01;
        if (path_length_table[destination] > (path_length_table[min_index] + distance)) {
            path_length_table[destination] = path_length_table[min_index] + distance;
            if (insert_val(path_length_table[destination], destination)) return 1;
        }
    }

    // таблица расстояний составлена - надо строить маршрут.
    // Берём зелёную клетку и выясняем в каком направлении есть ход.
    for (ii = north; ii <= west; ii++) {
        if (map[data.green_cell_nr].node_link[ii] != UNKNOWN) {
            prev_bearing = ii;
            min_index = (data.green_cell_nr << 1) | (ii & 0x01);
            break;
        }
    }
    data.pathlength = 0; // указатель пути на начало

    // Теперь скатываемся вниз для создания кратчайшего пути
    // от старта к финишу.
    // 1 line offset
    while (path_length_table[min_index]) {
        for (k = 0; k < 3; k++) {
            if (k < 2) {
                int step = 0;
                unsigned int next_min_index = min_index;
                while ((destination_map = map[next_min_index/2].node_link[2*k + (next_min_index & 0x01)]) != UNKNOWN) {
                    distance =  (next_min_index & 0x01) ? (map[min_index/2].coordinate.east  - map[destination_map].coordinate.east) :
                            (map[min_index/2].coordinate.north - map[destination_map].coordinate.north);
                    if (distance < 0) distance = -distance;
                    distance = TimeToRunStraight(distance);

                    // до этого destination_map был индекс в карте,
                    // а теперь destination индекс в таблице расстояний
                    destination = destination_map*2 + (next_min_index & 0x01);
                    if ((path_length_table[min_index] - distance) == (path_length_table[destination])) {
                        unsigned int map_idx = min_index / 2;
                        do {
                            bearing = (bearing_dir_t)((min_index & 0x01) + 2*k);
                            path[data.pathlength] = (rotation_dir_t)(bearing - prev_bearing);
                            path[data.pathlength] &= TURN_MASK;
                            prev_bearing = bearing;
                            destination_map = map[map_idx].node_link[bearing];
                            distance =  (next_min_index & 0x01) ? (map[map_idx].coordinate.east  - map[destination_map].coordinate.east) :
                                                                  (map[map_idx].coordinate.north - map[destination_map].coordinate.north);
                            if (distance < 0) distance = -distance;
                            if ((map_idx) != data.green_cell_nr) {
                                data.pathlength++;
                                length[data.pathlength] = distance;  // тут проблема возможно
                            } else {
                                length[data.pathlength] = distance ;  // тут проблема возможно
                            }
                            map_idx = destination_map;
                            min_index = destination;
//                          break;
                        } while (0 < step--);
                        break_flag = 1;
                    }
                    if (break_flag) { break; }
                    next_min_index = map[next_min_index/2].node_link[2*k + (next_min_index & 0x01)]*2 + (min_index & 0x01);
                    step++;
                }
                if (break_flag) { break_flag = 0; break; }
            } else {
                if ((path_length_table[min_index] - data.turncost) == (path_length_table[min_index ^ 0x01])) {
                    min_index = min_index ^ 0x01;
                    break;
                }
            }
        }
    }

    path[data.pathlength++] = back;
    calculation_time= benchmark_stop();

    copy_data_dma(path, data.path, data.pathlength);
	copy_data_dma((uint8_t *) length, (uint8_t *)data.length, (data.pathlength) * sizeof(length[0]));
	    UART0_OutString("Calculation time: ");
	    UART0_OutUDec(calculation_time);
	    UART0_OutString("/3 us\r\n");

	return 0;
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

/* **********************************************************************************
 * This is necessary for NEXT version of Maze robot, which will run on SQUARE MAZE
 * Only virtual moving.
 ************************************************************************************/

#include "square_maze.h"

bearing_dir_t real_bearing;
unsigned int my_position;
//coordinate_t real_coordinate;

void init_virtual_maze(void) {
    coordinate_t start = { //
    .east = (data.sq_init & 0x0F000) >>12, //{12, 5};
    .north= (data.sq_init & 0x00F00) >> 8,
    };
    unsigned int        maze_width = ((data.sq_init & 0x000F0) >> 4) + 1;
    unsigned int        maze_depth = ((data.sq_init & 0x0000F) >> 0) + 1;
    bearing_dir_t  start_direction = (bearing_dir_t)((data.sq_init & 0xF0000) >> 16);
    spi_read_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));
    init_maze(maze_width, maze_depth, start, start_direction);
//    draw_maze(start);

    my_position = data.green_cell_nr;
//    real_coordinate = map[my_position].coordinate;
    real_bearing = north; //(bearing_dir_t)((data.sq_init & 0x0030000) >> 16);
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
    return (distance + (data.cell_step/2))/data.cell_step;;
}

bearing_dir_t make_turn(bearing_dir_t turn_direction) {
    real_bearing += turn_direction;
    return (bearing_dir_t) (real_bearing &= TURN_MASK);
}

color_t get_color(void) {
    if (my_position == data.green_cell_nr) return green;
    if (my_position == data.red_cell_nr)    return red;
    return white;
}
