#include "maze.h"
#include "SPI_EEProm.h"
#include "dma.h"



const map_cell_t youtube_data[] = {
{0,0,11,1,UNKNOWN,UNKNOWN,0,0,4,4},
{600,0,13,2,UNKNOWN,0,0,0,4,0},
{900,0,14,3,UNKNOWN,1,0,0,4,0},
{1200,0,15,4,UNKNOWN,2,0,0,4,0},
{1500,0,16,5,UNKNOWN,3,0,0,4,0},
{1800,0,17,6,UNKNOWN,4,0,0,4,0},
{2100,0,18,UNKNOWN,UNKNOWN,5,0,4,4,0},
{2700,0,20,8,UNKNOWN,UNKNOWN,0,0,4,4},
{3300,0,21,9,UNKNOWN,7,0,0,4,0},
{3900,0,23,10,UNKNOWN,8,0,0,4,0},
{4200,0,UNKNOWN,UNKNOWN,UNKNOWN,9,4,4,4,0},
{0,300,25,12,0,UNKNOWN,0,0,0,4},
{300,300,26,UNKNOWN,UNKNOWN,11,0,4,4,0},
{600,300,27,14,1,UNKNOWN,0,0,0,4},
{900,300,28,UNKNOWN,2,13,0,4,0,0},
{1200,300,29,16,3,UNKNOWN,0,0,0,4},
{1500,300,30,17,4,15,0,0,0,0},
{1800,300,31,18,5,16,0,0,0,0},
{2100,300,45,19,6,17,0,0,0,0},
{2400,300,32,20,UNKNOWN,18,0,0,4,0},
{2700,300,33,21,7,19,0,0,0,0},
{3300,300,UNKNOWN,UNKNOWN,8,20,4,4,0,0},
{3600,300,36,23,UNKNOWN,UNKNOWN,0,0,4,4},
{3900,300,UNKNOWN,UNKNOWN,9,22,4,4,0,0},
{4200,300,38,UNKNOWN,UNKNOWN,UNKNOWN,0,4,4,4},
{0,600,UNKNOWN,26,11,UNKNOWN,4,0,0,4},
{300,600,40,27,12,25,0,0,0,0},
{600,600,41,28,13,26,0,0,0,0},
{900,600,42,29,14,27,0,0,0,0},
{1200,600,UNKNOWN,30,15,28,4,0,0,0},
{1500,600,44,31,16,29,0,0,0,0},
{1800,600,UNKNOWN,UNKNOWN,17,30,4,4,0,0},
{2400,600,UNKNOWN,33,19,UNKNOWN,4,0,0,4},
{2700,600,47,34,20,32,0,0,0,0},
{3000,600,UNKNOWN,UNKNOWN,UNKNOWN,33,4,4,4,0},
{3300,600,UNKNOWN,36,UNKNOWN,UNKNOWN,4,0,4,4},
{3600,600,49,UNKNOWN,22,35,0,4,0,0},
{3900,600,50,38,UNKNOWN,UNKNOWN,0,0,4,4},
{4200,600,89,UNKNOWN,24,37,0,4,0,0},
{0,900,64,40,UNKNOWN,UNKNOWN,0,0,4,4},
{300,900,UNKNOWN,41,26,39,4,0,0,0},
{600,900,52,UNKNOWN,27,40,0,4,0,0},
{900,900,53,43,28,UNKNOWN,0,0,0,4},
{1200,900,54,44,UNKNOWN,42,0,0,4,0},
{1500,900,55,45,30,43,0,0,0,0},
{2100,900,57,46,18,44,0,0,0,0},
{2400,900,UNKNOWN,UNKNOWN,UNKNOWN,45,4,4,4,0},
{2700,900,UNKNOWN,48,33,UNKNOWN,4,0,0,4},
{3000,900,60,UNKNOWN,UNKNOWN,47,0,4,4,0},
{3600,900,UNKNOWN,50,36,UNKNOWN,4,0,0,4},
{3900,900,62,UNKNOWN,37,49,0,4,0,0},
{300,1200,65,UNKNOWN,UNKNOWN,UNKNOWN,0,4,4,4},
{600,1200,UNKNOWN,53,41,UNKNOWN,4,0,0,4},
{900,1200,67,54,42,52,0,0,0,0},
{1200,1200,68,55,43,53,0,0,0,0},
{1500,1200,UNKNOWN,56,44,54,4,0,0,0},
{1800,1200,70,UNKNOWN,UNKNOWN,55,0,4,4,0},
{2100,1200,71,58,45,UNKNOWN,0,0,0,4},
{2400,1200,72,59,UNKNOWN,57,0,0,4,0},
{2700,1200,73,60,UNKNOWN,58,0,0,4,0},
{3000,1200,UNKNOWN,UNKNOWN,48,59,4,4,0,0},
{3300,1200,85,62,UNKNOWN,UNKNOWN,0,0,4,4},
{3900,1200,UNKNOWN,63,50,61,4,0,0,0},
{4200,1200,88,UNKNOWN,UNKNOWN,62,0,4,4,0},
{0,1500,77,65,39,UNKNOWN,0,0,0,4},
{300,1500,78,UNKNOWN,51,64,0,4,0,0},
{600,1500,79,67,UNKNOWN,UNKNOWN,0,0,4,4},
{900,1500,80,68,53,66,0,0,0,0},
{1200,1500,81,69,54,67,0,0,0,0},
{1500,1500,UNKNOWN,UNKNOWN,UNKNOWN,68,4,4,4,0},
{1800,1500,82,71,56,UNKNOWN,0,0,0,4},
{2100,1500,UNKNOWN,UNKNOWN,57,70,4,4,0,0},
{2400,1500,83,73,58,UNKNOWN,0,0,0,4},
{2700,1500,84,74,59,72,0,0,0,0},
{3000,1500,UNKNOWN,UNKNOWN,UNKNOWN,73,4,4,4,0},
{3600,1500,86,UNKNOWN,UNKNOWN,UNKNOWN,0,4,4,4},
{3900,1500,87,UNKNOWN,UNKNOWN,UNKNOWN,0,4,4,4},
{0,1800,UNKNOWN,UNKNOWN,64,UNKNOWN,4,4,0,4},
{300,1800,UNKNOWN,79,65,UNKNOWN,4,0,0,4},
{600,1800,UNKNOWN,80,66,78,4,0,0,0},
{900,1800,UNKNOWN,UNKNOWN,67,79,4,4,0,0},
{1200,1800,UNKNOWN,82,68,UNKNOWN,4,0,0,4},
{1800,1800,UNKNOWN,83,70,81,4,0,0,0},
{2400,1800,UNKNOWN,UNKNOWN,72,82,4,4,0,0},
{2700,1800,UNKNOWN,UNKNOWN,73,UNKNOWN,4,4,0,4},
{3300,1800,UNKNOWN,86,61,UNKNOWN,4,0,0,4},
{3600,1800,UNKNOWN,87,75,85,4,0,0,0},
{3900,1800,UNKNOWN,88,76,86,4,0,0,0},
{4200,1800,UNKNOWN,UNKNOWN,63,87,4,4,0,0},
{4200,900,UNKNOWN,UNKNOWN,38,UNKNOWN,4,4,0,4}
};

const map_cell_t alt_data[] = {
{0, 0, 1, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{0, 300, UNKNOWN, UNKNOWN, 0, 12, 4, 4, 0, 0},
{0, 600, 3, UNKNOWN, UNKNOWN, 13, 0, 4, 4, 0},
{0, 900, UNKNOWN, UNKNOWN, 2, UNKNOWN, 4, 4, 0, 4},
{0, 1200, 5, UNKNOWN, UNKNOWN, 15, 0, 4, 4, 0},
{0, 1800, UNKNOWN, UNKNOWN, 4, 17, 4, 4, 0, 0},
{0, 2100, UNKNOWN, UNKNOWN, UNKNOWN, 18, 4, 4, 4, 0},
{0, 2400, UNKNOWN, UNKNOWN, UNKNOWN, 19, 4, 4, 4, 0},
{0, 2700, 9, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{0, 3000, 10, UNKNOWN, 8, 21, 0, 4, 0, 0},
{0, 3600, UNKNOWN, UNKNOWN, 9, 23, 4, 4, 0, 0},
{0, 3900, UNKNOWN, UNKNOWN, UNKNOWN, 31, 4, 4, 4, 0},
{-300, 300, 13, 1, UNKNOWN, 24, 0, 0, 4, 0},
{-300, 600, UNKNOWN, 2, 12, UNKNOWN, 4, 0, 0, 4},
{-300, 900, 15, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{-300, 1200, UNKNOWN, 4, 14, 25, 4, 0, 0, 0},
{-300, 1500, 17, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{-300, 1800, 18, 5, 16, 27, 0, 0, 0, 0},
{-300, 2100, 19, 6, 17, UNKNOWN, 0, 0, 0, 4},
{-300, 2400, UNKNOWN, 7, 18, UNKNOWN, 4, 0, 0, 4},
{-300, 2700, 21, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{-300, 3000, UNKNOWN, 9, 20, 28, 4, 0, 0, 0},
{-300, 3300, 23, UNKNOWN, UNKNOWN, 29, 0, 4, 4, 0},
{-300, 3600, UNKNOWN, 10, 22, 30, 4, 0, 0, 0},
{-600, 300, 25, 12, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-600, 1200, 26, 15, 24, UNKNOWN, 0, 0, 0, 4},
{-600, 1500, UNKNOWN, UNKNOWN, 25, UNKNOWN, 4, 4, 0, 4},
{-600, 1800, 28, 17, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-600, 3000, UNKNOWN, 21, 27, UNKNOWN, 4, 0, 0, 4},
{-600, 3300, UNKNOWN, 22, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-600, 3600, 31, 23, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-600, 3900, UNKNOWN, 11, 30, 34, 4, 0, 0, 0},
{-600, 0, UNKNOWN, UNKNOWN, UNKNOWN, 33, 4, 4, 4, 0},
{-900, 0, 34, 32, UNKNOWN, 35, 0, 0, 4, 0},
{-900, 3900, UNKNOWN, 31, 33, UNKNOWN, 4, 0, 0, 4},
{-1200, 0, 36, 33, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-1200, 300, 37, UNKNOWN, 35, 48, 0, 4, 0, 0},
{-1200, 900, UNKNOWN, UNKNOWN, 36, UNKNOWN, 4, 4, 0, 4},
{-1200, 1200, UNKNOWN, UNKNOWN, UNKNOWN, 51, 4, 4, 4, 0},
{-1200, 1500, 40, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{-1200, 1800, 41, UNKNOWN, 39, 53, 0, 4, 0, 0},
{-1200, 2400, UNKNOWN, UNKNOWN, 40, 55, 4, 4, 0, 0},
{-1200, 2700, UNKNOWN, UNKNOWN, UNKNOWN, 56, 4, 4, 4, 0},
{-1200, 3000, 44, UNKNOWN, UNKNOWN, 57, 0, 4, 4, 0},
{-1200, 3300, UNKNOWN, UNKNOWN, 43, UNKNOWN, 4, 4, 0, 4},
{-1200, 3600, 46, UNKNOWN, UNKNOWN, 59, 0, 4, 4, 0},
{-1200, 3900, UNKNOWN, UNKNOWN, 45, UNKNOWN, 4, 4, 0, 4},
{-1500, 0, 48, UNKNOWN, UNKNOWN, UNKNOWN, 0, 4, 4, 4},
{-1500, 300, 49, 36, 47, 61, 0, 0, 0, 0},
{-1500, 600, 50, UNKNOWN, 48, 62, 0, 4, 0, 0},
{-1500, 900, 51, UNKNOWN, 49, 63, 0, 4, 0, 0},
{-1500, 1200, 52, 38, 50, 64, 0, 0, 0, 0},
{-1500, 1500, 53, UNKNOWN, 51, 65, 0, 4, 0, 0},
{-1500, 1800, UNKNOWN, 40, 52, 66, 4, 0, 0, 0},
{-1500, 2100, UNKNOWN, UNKNOWN, UNKNOWN, 67, 4, 4, 4, 0},
{-1500, 2400, 56, 41, UNKNOWN, 68, 0, 0, 4, 0},
{-1500, 2700, 57, 42, 55, 69, 0, 0, 0, 0},
{-1500, 3000, 58, 43, 56, UNKNOWN, 0, 0, 0, 4},
{-1500, 3300, 59, UNKNOWN, 57, 71, 0, 4, 0, 0},
{-1500, 3600, 60, 45, 58, UNKNOWN, 0, 0, 0, 4},
{-1500, 3900, UNKNOWN, UNKNOWN, 59, 73, 4, 4, 0, 0},
{-1800, 300, UNKNOWN, 48, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-1800, 600, UNKNOWN, 49, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-1800, 900, UNKNOWN, 50, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-1800, 1200, UNKNOWN, 51, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-1800, 1500, UNKNOWN, 52, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-1800, 1800, 67, 53, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-1800, 2100, UNKNOWN, 54, 66, UNKNOWN, 4, 0, 0, 4},
{-1800, 2400, UNKNOWN, 55, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
{-1800, 2700, 70, 56, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-1800, 3000, UNKNOWN, UNKNOWN, 69, UNKNOWN, 4, 4, 0, 4},
{-1800, 3300, 72, 58, UNKNOWN, UNKNOWN, 0, 0, 4, 4},
{-1800, 3600, UNKNOWN, UNKNOWN, 71, UNKNOWN, 4, 4, 0, 4},
{-1800, 3900, UNKNOWN, 60, UNKNOWN, UNKNOWN, 4, 0, 4, 4},
};

typedef struct {
    const map_cell_t *    pre_map_data;
    int             sq_data;
    int             map_size;
    int             map_start_cell;
    int             map_end_cell;
    int             maxX;
    int             maxY;
    char            map_name[16];
} alt_data_t;

alt_data_t  inject_map[] = {
    {alt_data, 0x00100d6, sizeof(alt_data)/sizeof(alt_data[0]),                  0, 73, 13, 6, "Sigulda         "},
    {youtube_data, 0x0000c5e6, sizeof(youtube_data)/sizeof(youtube_data[0]),    75, 51, 13, 7, "YouTube         "},
};
unsigned int inject(unsigned int select) {
//      data_ptr = (uint32_t*) alt_data;
//      unsigned int *data_ptr;
//		Flash_Erase(0x30000);
//		Flash_Erase(0x31000);
//		Flash_WriteArray(data_ptr, ROM_map_addr, sizeof(alt_data)/4);
    spi_write_eeprom(ROM_map_addr, (unsigned char *) inject_map[select].pre_map_data, inject_map[select].map_size*sizeof(map_cell_t));
    data.map_size       = inject_map[select].map_size;
    data.red_cell_nr    = inject_map[select].map_end_cell;
    data.green_cell_nr  = inject_map[select].map_start_cell;
    data.pathlength     = 0;
    data.sq_init        = inject_map[select].sq_data;
    return 0;
}

// Функция конвертирует данные из программы winmaze в формат лабиринта из линий
// Каждой клетке в соответствие ставится узел, кроме "туннельных" клеток (0x0A и 0x05).
// Для приведения соответствия используется массив linkarray. Но так как он заполняется
// по мере обработки клеточек (с юга на север и с запада на восток), в нем нет данных
// о будущих узлах. Поэтому линки с запада на восток заполняеются по достижении восточной
// клетки. клетка с севера всегда будет на 1 больше текущей. Для обеспечения такого
// функционала "туннельным" клеткам присваивается индекс клетки находящейся южнее, если
// стенки с запада и востока (0x0A) или западной, если стенки с севера и юга.
//
//   | 2   9 | 12| 20|
//   +   +---+   +---+
//   | 1   1   12  19|
//   |   +-------+   |
//   | 0 | 7   7   18|
//   |   +   +---+   |
//   | 0 | 6 | 11  17|
//   +---+---+---+---+---

unsigned int  convert_maze(uint8_t * mazfile) {
    unsigned int ii, jj, index = 0;
    uint8_t linkarray[16*16];
    map_cell_t *map_ptr;
    map_ptr = map;
    for (jj = 0; jj < 16; jj++) {
        for (ii = 0; ii < 16; ii++){
            if (((*mazfile & 0x0F) == 0x0a) && ii) {
                linkarray[ii+(jj*16)] = linkarray[ii-1 + (jj*16)];
                mazfile++;
                continue;
            }
            if (((*mazfile & 0x0F) == 0x05) && jj) {
                linkarray[ii+(jj*16)] = linkarray[ii + ((jj-1)*16)];
                mazfile++;
                continue;
            }

            linkarray[ii+(jj*16)] = index;
            map_ptr->coordinate.north = data.cell_step * ii;
            map_ptr->coordinate.east  = data.cell_step * jj;
// north
            if (*mazfile & 0x01) {
                map_ptr->node_link[0] = UNKNOWN;
                map_ptr->pass_count[0] = 4;
            } else {
                map_ptr->node_link[0] = index+1;
                map_ptr->pass_count[0] = 0;
            }
// east
            if (*mazfile & 0x02) {
                map_ptr->node_link[1] = UNKNOWN;
                map_ptr->pass_count[1] = 4;
            } else {
                map_ptr->node_link[1] = linkarray[ii + ((jj+1) * 16)]; // error
                map_ptr->pass_count[1] = 0;
            }
//south
            if (*mazfile & 0x04) {
                map_ptr->node_link[2] = UNKNOWN;
                map_ptr->pass_count[2] = 4;
            } else {
                map_ptr->node_link[2] = linkarray[ii-1 + (jj * 16)];
                map_ptr->pass_count[2] = 0;
                if (ii) {
                    map[linkarray[ii-1 + (jj*16)]].node_link[0] = index;
                }
            }
// west
            if (*mazfile & 0x08) {
                map_ptr->node_link[3] = UNKNOWN;
                map_ptr->pass_count[3] = 4;
            } else {
                map_ptr->node_link[3] = linkarray[ii + ((jj-1) * 16)];
                map_ptr->pass_count[3] = 0;
                if (jj) {
                    map[linkarray[ii + ((jj-1)*16)]].node_link[1] = index;
                }
            }
            mazfile++;
            map_ptr++;
            if (++index > MAX_MAP_SIZE) { // лабиринт не влезает в отведенную память
                data.map_size = 0;
                data.pathlength = 0;
                return 1;
            }
        }
    }
    spi_write_eeprom(ROM_map_addr, (unsigned char *) map, sizeof(map_cell_t)*index);
    data.map_size       = index;
    data.red_cell_nr    = 0;
    data.green_cell_nr  = 0;
    data.pathlength     = 0;
    data.sq_init        = 0x000000FF;
    return 0;
}

// array size is 256
const uint8_t APEC88[]  = {
  0x0e, 0x0a, 0x08, 0x08, 0x0a, 0x0a, 0x0a, 0x08, 0x0a, 0x08, 0x08, 0x0a, 0x0a, 0x0a, 0x08, 0x09,
  0x0c, 0x0a, 0x03, 0x06, 0x09, 0x0e, 0x08, 0x02, 0x0a, 0x01, 0x05, 0x0e, 0x0a, 0x0a, 0x03, 0x05,
  0x06, 0x0a, 0x09, 0x0c, 0x03, 0x0c, 0x02, 0x09, 0x0c, 0x03, 0x07, 0x0c, 0x0a, 0x0a, 0x08, 0x03,
  0x0c, 0x08, 0x01, 0x06, 0x09, 0x06, 0x09, 0x06, 0x03, 0x0c, 0x0a, 0x00, 0x0a, 0x0a, 0x02, 0x09,
  0x05, 0x05, 0x05, 0x0d, 0x04, 0x0b, 0x06, 0x09, 0x0e, 0x01, 0x0d, 0x06, 0x08, 0x08, 0x09, 0x05,
  0x06, 0x03, 0x04, 0x02, 0x02, 0x09, 0x0e, 0x02, 0x0a, 0x03, 0x04, 0x09, 0x05, 0x05, 0x07, 0x05,
  0x0c, 0x09, 0x06, 0x0a, 0x08, 0x03, 0x0c, 0x08, 0x0a, 0x09, 0x05, 0x05, 0x05, 0x06, 0x09, 0x05,
  0x05, 0x05, 0x0c, 0x09, 0x04, 0x09, 0x05, 0x04, 0x09, 0x05, 0x05, 0x04, 0x03, 0x0c, 0x03, 0x05,
  0x04, 0x02, 0x03, 0x06, 0x03, 0x07, 0x05, 0x06, 0x03, 0x05, 0x05, 0x05, 0x0d, 0x05, 0x0e, 0x03,
  0x04, 0x09, 0x0c, 0x0a, 0x0a, 0x0b, 0x06, 0x08, 0x0a, 0x01, 0x05, 0x07, 0x05, 0x06, 0x0a, 0x09,
  0x07, 0x05, 0x04, 0x0b, 0x0c, 0x0a, 0x09, 0x05, 0x0d, 0x05, 0x06, 0x0a, 0x00, 0x0a, 0x0a, 0x03,
  0x0d, 0x04, 0x03, 0x0c, 0x03, 0x0d, 0x06, 0x03, 0x05, 0x04, 0x09, 0x0d, 0x06, 0x0a, 0x0a, 0x09,
  0x04, 0x03, 0x0c, 0x03, 0x0c, 0x01, 0x0d, 0x0d, 0x06, 0x01, 0x05, 0x04, 0x09, 0x0c, 0x08, 0x03,
  0x04, 0x09, 0x04, 0x0a, 0x03, 0x04, 0x00, 0x02, 0x0b, 0x05, 0x05, 0x05, 0x05, 0x05, 0x04, 0x09,
  0x05, 0x05, 0x06, 0x0a, 0x0a, 0x03, 0x06, 0x0a, 0x09, 0x06, 0x00, 0x03, 0x06, 0x02, 0x03, 0x05,
  0x06, 0x02, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x02, 0x0b, 0x06, 0x0a, 0x0a, 0x0a, 0x0a, 0x03
};

const unsigned char world1st[256] ={
    0x0e,0x0a,0x09,0x0c,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x08,0x0a,0x0a,0x0a,0x08,0x09,
    0x0c,0x09,0x05,0x06,0x08,0x0a,0x0a,0x0a,0x0a,0x0b,0x06,0x0a,0x0a,0x0a,0x03,0x05,
    0x05,0x05,0x05,0x0c,0x02,0x0b,0x0e,0x08,0x0a,0x0a,0x08,0x0a,0x08,0x08,0x09,0x05,
    0x05,0x04,0x01,0x06,0x08,0x0a,0x09,0x04,0x0a,0x0a,0x00,0x0a,0x03,0x05,0x05,0x05,
    0x05,0x05,0x04,0x09,0x06,0x09,0x05,0x04,0x0a,0x0a,0x02,0x0a,0x0b,0x05,0x05,0x05,
    0x05,0x04,0x03,0x06,0x0a,0x02,0x03,0x06,0x0a,0x0a,0x0a,0x0a,0x09,0x05,0x05,0x05,
    0x05,0x05,0x0d,0x0d,0x0d,0x0c,0x08,0x0a,0x0a,0x0a,0x0a,0x09,0x05,0x05,0x05,0x05,
    0x06,0x03,0x04,0x01,0x04,0x01,0x05,0x0c,0x09,0x0c,0x08,0x01,0x05,0x05,0x05,0x05,
    0x0c,0x08,0x01,0x06,0x01,0x05,0x04,0x02,0x03,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x0d,0x06,0x01,0x05,0x0c,0x0a,0x01,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x04,0x09,0x06,0x03,0x06,0x0a,0x02,0x00,0x03,0x05,0x04,0x03,0x05,
    0x05,0x04,0x03,0x05,0x05,0x0c,0x0a,0x0a,0x08,0x09,0x04,0x0a,0x01,0x05,0x0d,0x05,
    0x05,0x05,0x0d,0x05,0x05,0x04,0x0a,0x08,0x03,0x05,0x06,0x0a,0x03,0x05,0x04,0x01,
    0x05,0x05,0x04,0x01,0x04,0x03,0x0c,0x02,0x0b,0x06,0x08,0x0a,0x0a,0x03,0x05,0x05,
    0x05,0x06,0x01,0x07,0x06,0x08,0x02,0x0a,0x0a,0x0b,0x06,0x08,0x0a,0x0a,0x00,0x01,
    0x06,0x0a,0x02,0x0a,0x0a,0x02,0x0b,0x0e,0x0a,0x0a,0x0a,0x02,0x0a,0x0a,0x03,0x07
};

const unsigned char kankou2003[256] ={
    0x0e,0x0a,0x0a,0x08,0x0a,0x0a,0x0a,0x08,0x0a,0x0a,0x08,0x0a,0x08,0x0a,0x08,0x09,
    0x0c,0x0a,0x08,0x02,0x0a,0x0b,0x0f,0x05,0x0e,0x0b,0x06,0x09,0x06,0x09,0x05,0x05,
    0x04,0x09,0x05,0x0c,0x0b,0x0e,0x0a,0x02,0x08,0x0a,0x0b,0x04,0x0b,0x04,0x01,0x05,
    0x05,0x05,0x05,0x04,0x0a,0x0a,0x0a,0x09,0x05,0x0c,0x0a,0x03,0x0f,0x05,0x06,0x03,
    0x05,0x05,0x05,0x06,0x0b,0x0f,0x0d,0x07,0x04,0x03,0x0e,0x08,0x0a,0x03,0x0f,0x0d,
    0x05,0x05,0x06,0x0a,0x08,0x0a,0x02,0x0a,0x03,0x0e,0x0b,0x06,0x0a,0x0a,0x0a,0x01,
    0x05,0x06,0x0a,0x09,0x06,0x0a,0x08,0x0b,0x0c,0x0a,0x09,0x0c,0x0a,0x0a,0x09,0x05,
    0x04,0x08,0x09,0x06,0x0a,0x09,0x05,0x0c,0x01,0x0f,0x05,0x07,0x0e,0x09,0x05,0x05,
    0x05,0x05,0x04,0x0a,0x09,0x05,0x05,0x06,0x03,0x0f,0x05,0x0d,0x0e,0x03,0x05,0x05,
    0x05,0x05,0x05,0x0f,0x05,0x05,0x06,0x09,0x0f,0x0c,0x03,0x06,0x0a,0x0a,0x03,0x05,
    0x05,0x07,0x04,0x0a,0x03,0x06,0x08,0x02,0x09,0x06,0x08,0x0b,0x0e,0x0a,0x0a,0x01,
    0x04,0x0b,0x05,0x0c,0x0b,0x0f,0x07,0x0d,0x04,0x09,0x06,0x0a,0x0a,0x09,0x0d,0x05,
    0x06,0x09,0x05,0x04,0x0a,0x0a,0x0a,0x03,0x07,0x06,0x0a,0x09,0x0f,0x05,0x06,0x01,
    0x0f,0x05,0x05,0x06,0x0b,0x0e,0x0a,0x0a,0x0a,0x0a,0x0b,0x04,0x0b,0x04,0x09,0x05,
    0x0c,0x03,0x06,0x0a,0x09,0x0d,0x0c,0x0a,0x0a,0x0a,0x0a,0x03,0x0c,0x03,0x06,0x01,
    0x06,0x0a,0x0a,0x0a,0x02,0x02,0x02,0x0a,0x0a,0x0a,0x0a,0x0a,0x02,0x0a,0x0a,0x03
};

const unsigned char russian2004[256] ={
    0x0e,0x0a,0x0a,0x0a,0x09,0x0c,0x0a,0x0a,0x09,0x0c,0x0a,0x0a,0x0a,0x0a,0x0a,0x09,
    0x0c,0x0a,0x0a,0x0a,0x02,0x02,0x0a,0x09,0x04,0x02,0x08,0x0a,0x09,0x0c,0x09,0x05,
    0x05,0x0c,0x0a,0x0a,0x0a,0x0a,0x09,0x05,0x05,0x0c,0x02,0x09,0x06,0x03,0x06,0x01,
    0x05,0x06,0x08,0x0a,0x0a,0x09,0x05,0x05,0x05,0x07,0x0c,0x03,0x0c,0x0a,0x08,0x01,
    0x06,0x09,0x05,0x0c,0x09,0x05,0x05,0x05,0x05,0x0c,0x01,0x0c,0x02,0x09,0x05,0x05,
    0x0c,0x01,0x05,0x05,0x05,0x05,0x06,0x03,0x06,0x01,0x05,0x07,0x0e,0x02,0x01,0x07,
    0x05,0x05,0x05,0x05,0x05,0x04,0x09,0x0e,0x0a,0x03,0x06,0x08,0x09,0x0c,0x02,0x09,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x0c,0x08,0x0a,0x09,0x05,0x05,0x05,0x0e,0x01,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x06,0x03,0x0d,0x05,0x05,0x05,0x06,0x08,0x01,
    0x05,0x06,0x03,0x05,0x05,0x05,0x04,0x0a,0x0a,0x03,0x05,0x05,0x04,0x09,0x05,0x05,
    0x04,0x0a,0x0a,0x03,0x04,0x03,0x05,0x0c,0x0a,0x08,0x01,0x05,0x05,0x05,0x05,0x05,
    0x04,0x09,0x0c,0x09,0x05,0x0c,0x01,0x05,0x0c,0x03,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x06,0x09,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x05,0x05,0x05,0x06,0x03,0x0c,0x03,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x06,0x03,0x04,0x02,0x02,0x0a,0x0a,0x02,0x09,0x05,0x06,0x03,0x06,0x03,0x05,
    0x06,0x0a,0x0a,0x02,0x0a,0x0a,0x0a,0x0a,0x0a,0x03,0x06,0x0a,0x0a,0x0a,0x0a,0x03
};

const unsigned char EURO_V59[256] ={
     0x0e,0x0a,0x0a,0x09,0x0c,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x09,0x0c,0x09,
     0x0c,0x0a,0x0a,0x01,0x05,0x0c,0x09,0x0c,0x0a,0x0a,0x0a,0x0a,0x09,0x05,0x05,0x05,
     0x05,0x0c,0x09,0x05,0x06,0x03,0x05,0x05,0x0c,0x09,0x0c,0x0a,0x03,0x06,0x03,0x05,
     0x05,0x05,0x05,0x06,0x0a,0x0a,0x03,0x05,0x05,0x05,0x06,0x0a,0x0a,0x09,0x0c,0x03,
     0x06,0x03,0x05,0x0c,0x0a,0x0a,0x0a,0x02,0x01,0x06,0x0a,0x08,0x09,0x05,0x06,0x09,
     0x0c,0x0a,0x03,0x05,0x0c,0x0a,0x0a,0x0a,0x03,0x0c,0x09,0x05,0x05,0x06,0x09,0x05,
     0x06,0x09,0x0c,0x03,0x05,0x0c,0x08,0x0a,0x0a,0x01,0x05,0x06,0x02,0x09,0x05,0x05,
     0x0c,0x03,0x05,0x0c,0x03,0x05,0x05,0x0c,0x09,0x05,0x04,0x08,0x0a,0x03,0x05,0x05,
     0x05,0x0c,0x01,0x05,0x0c,0x03,0x05,0x04,0x03,0x05,0x05,0x05,0x0c,0x09,0x05,0x05,
     0x05,0x05,0x05,0x05,0x06,0x09,0x06,0x02,0x0a,0x03,0x05,0x05,0x05,0x06,0x02,0x01,
     0x05,0x05,0x05,0x06,0x08,0x02,0x0a,0x0a,0x0a,0x0a,0x03,0x05,0x07,0x0c,0x0a,0x03,
     0x05,0x05,0x06,0x0b,0x06,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x02,0x0a,0x01,0x0c,0x09,
     0x05,0x06,0x0a,0x0a,0x0a,0x09,0x0c,0x09,0x0c,0x0a,0x0a,0x09,0x0d,0x05,0x05,0x05,
     0x06,0x09,0x0c,0x0a,0x08,0x01,0x05,0x05,0x06,0x08,0x0a,0x03,0x05,0x06,0x03,0x05,
     0x0c,0x03,0x06,0x09,0x05,0x06,0x03,0x06,0x0a,0x02,0x0a,0x0a,0x02,0x08,0x09,0x05,
     0x06,0x0a,0x0a,0x03,0x06,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x0a,0x03,0x06,0x03
};

void map_select(void) {
    const menuitem_t map_menu_items[] = {
        {"Erase map       ", 0,  none}, // 0
        {"APEC 88         ", 0,  none}, // 1
        {"1st World       ", 0,  none}, // 2
        {"2003 Kankou     ", 0,  none}, // 3
        {"2004 Russia     ", 0,  none}, // 4
        {"Euro-V59        ", 0,  none}, // 5
        {"Sigulda         ", 0,  none}, // 6
        {"YouTube         ", 0,  none}, // 7
    };
    switch (do_menu((menuitem_t*) map_menu_items, ((sizeof(map_menu_items)/sizeof(map_menu_items[0]))-1))) {
    case 0:
        data.map_size       = 0;
        data.pathlength     = 0;
        data.green_cell_nr  = 0;
        data.red_cell_nr    = 0;
        break;

    case 1:
        convert_maze((uint8_t *)APEC88);
        break;

    case 2:
        convert_maze((uint8_t *) world1st);
        break;

    case 3:
        convert_maze((uint8_t *)kankou2003);
        break;

    case 4:
        convert_maze((uint8_t *)russian2004);
        break;

    case 5:
        convert_maze((uint8_t *)EURO_V59);
        break;

    case 6:
        inject(0);
        break;

    case 7:
        inject(1);
        break;

    default:
        break;
    }
}
