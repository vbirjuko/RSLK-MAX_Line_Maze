/*
 * Logging.h
 *
 *  Created on: 29 нояб. 2020 г.
 *      Author: wl
 */

#ifndef LOGGING_H_
#define LOGGING_H_

void data_log(uint16_t datatowrite, unsigned int write_enable);
void data_log_init(void);
void data_log_finish(void);

enum where {
    node_idx = 0x00,
    Solve = 0x01,               // frame related
    Segment = 0x02,             // frame related
    Turn = 0x03,                // frame related
    coord_east_lsb = 0x10,
    coord_east_msb = 0x20,
    coord_north_lsb = 0x30,
    coord_north_msb = 0x40,
    segm_length_lsb = 0x50,
    segm_length_msb = 0x60,
};

/*
data_log(where_am_i << 8 | current_sensor, 1);
data_log( (my_coordinate.east & 0x000000ff)         | (1 << 12), 1);
data_log( ((my_coordinate.east & 0x0000ff00) >> 8)  | (2 << 12), 1);
data_log( (my_coordinate.north & 0x000000ff)        | (3 << 12), 1);
data_log( ((my_coordinate.north & 0x0000ff00)>> 8)  | (4 << 12), 1);
data_log( (segment_length & 0x000000ff)             | (5 << 12), 1);
data_log( ((segment_length & 0x0000ff00)>> 8)       | (6 << 12), 1);
data_log( current_index & 0xff , 1);
*/
#define STRAIGHT_MASK   (1 << 5)
#define LEFT_MASK       (1 << 6)
#define RIGHT_MASK      (1 << 7)


#endif /* LOGGING_H_ */
