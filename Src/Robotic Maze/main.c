// define RSLK_MAX SSD1306 PCA WITH_BGX BLINKER_SEGMENT UPSIDEDOWN



#include "msp.h"
#include "main.h"
#include "keyboard.h"
#include "Adafruit_ssd1306.h"
#include "display.h"
#include "i2c_drv.h"
#include "BumpInt.h"
#include "Motor.h"
#include "Tachometer.h"
#include "Reflectance.h"
#include "Clock.h"
#include "SysTickInts.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "Blinker.h"
#include "UART_drv.h"
#include "UART0.h"
#include "configure.h"
#include "color_sensor.h"
#include "Maze.h"
#include "Port_mapper.h"
#include "rand.h"
#ifdef WITH_SNP_
    #include "BLE_cc2560.h"
    #include "AP.h"
#endif
#include "SPI_EEProm.h"
#include "UART1.h"
#include "commandline.h"
#include "ADC.h"
#include "dma.h"
#include "inject_data.h"
#include "Logging.h"
#include "crc.h"
#include "square_maze.h"
#include "Timer32.h"
#include "FRAM_Logging.h"

volatile unsigned int time, time_delay;
void Solve_Maze(void);
void Configure(void);
void TestReflect(void);
void TestColor(void);
void TestBump(void);
void TestMotor(void);
void TestTachom(void);
void TestBrake(void);
void DrawMap(void);
void Explore_Maze(void);
void DumpMemory(void);
void SearchShortWay(void);
void ShowBattery(void);
void test_memory(void);
void SpeedPlay(void);
void test_sq_maze(void);


void SysTick_Handler(void){ // every 1/2 ms
	time++;
	if (kbddelay) kbddelay--;
	if (time_delay) time_delay--;
	ADC14->CTL0 |= ADC14_CTL0_SC;
}



//static uint32_t *data_ptr, *src_ptr;

/*
 * main.c
 */
int main(void){
	unsigned int crc_err = 0;

	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

//    Clock_Init48MHz();
    Clock_Init();
    Timer32_Init();
    SysTick_Init(48000000u/SCANPERSECOND, 2);
	Port_Mapper_Init();
	DMA_Init();
	LaunchPad_Init();
	LaunchPad_LED(0);
	SPI_EEProm_Init();

    spi_read_eeprom(EEPROM_CONFIG_ADDRESS, (unsigned char *)&data, sizeof(data));
    crc_err = calc_crc32((uint8_t*)&data, sizeof(data));
    if (crc_err){
        LaunchPad_LED(1);
    }

	if (LaunchPad_Input() == 0x03) {
	    inject(2);
	    while (LaunchPad_Input() ) continue;
	}

	Seed(data.lefthand);
	ADC0_InitSWTriggerCh21();
	i2c_master_init();
	kbd_init();
	Bump_Init();
	Motor_Init();
	color_sensor_init();
	Reflectance_Init_with_Timer(data.threshold);
	UART0_Init();
	FRAM_Logging_Init();
	display_init();
	UART1_Init();
#ifdef RSLK_MAX
	Blinker_Init();
#endif
#ifdef WITH_SNP
		BLE_Init();
#endif

	if (crc_err) {
	    crc_err = 0;
	    squareXY(0, 0, 127, 63, 0);
        putstr(2, 2, "EEPROM read", 0);
        putstr(2, 3, " CRC error", 0);
        update_display();
        while (kbdread() != KEY_DOWN) continue;
	}

	while(1){
		const menuitem_t main_menu_items[] = {
		    {"test square     ", test_sq_maze,  execute},
		    {"Speed Play      ", SpeedPlay,     execute},
            {"Solve Maze      ", Solve_Maze, 	execute},
			{"Explore Maze    ", Explore_Maze,  execute},
			{"Map draw        ", DrawMap, 		execute},
			{"Search Short Way", SearchShortWay, execute},
			{"Configure       ", Configure, 	execute},
			{"Battery Voltage ", ShowBattery,	execute},
			{"Reflectance test", TestReflect,   execute},
			{"Color sens. test", TestColor, 	execute},
			{"Bump sensor test", TestBump, 		execute},
			{"Motor test      ", TestMotor, 	execute},
			{"Tachometer  test", TestTachom, 	execute},
			{"Free Run        ", FreeRun, 		execute},
		};
		do_menu((menuitem_t*) main_menu_items, ((sizeof(main_menu_items)/sizeof(main_menu_items[0]))-1));
	}
}

void Solve_Maze(void) {
    volatile unsigned int record_count = 0;
    show_number(data.runnumber, 0);
    FRAM_log_Start(0x0004);
    frames_to_go = FRAM_SIZE/sizeof(data_buffer_t);
    delay_us(1000000);
    time = 0;
    Motor_Enable();
    if (solveMaze(0) == 0) {
        show_number((time+(SCANPERSECOND/20))/(SCANPERSECOND/10), 1);
    }
    where_am_i = 0;
    data_log_finish();
    LaunchPad_Output(0);
    if (FRAM_log_Stop()) LaunchPad_Output(RED);
    record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
    if (FRAM_log_Start(0x0000)) LaunchPad_Output(RED|GREEN);
    if (FRAM_log_write((uint8_t*)&record_count, ((void* )0 ), sizeof(record_count))) LaunchPad_Output(RED|BLUE);
    if (FRAM_log_Stop()) LaunchPad_Output(BLUE);
//    putstr(0, 6, "Press UP to save", 0);
//    putstr(0, 7,  " map in EEPROM ", 0);
    frames_to_go = 0;
    Motor_Speed(0, 0);
    while (kbdread() != KEY_DOWN) continue;

//    do {
//        unsigned int keyb;
//        keyb = kbdread();
//        if (keyb == KEY_DOWN) break;
//        if (keyb == KEY_UP) {
//            spi_write_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));
//            break;
//        }
//    } while(1);
    Motor_Disable();
}

void SpeedPlay(void) {
    show_number(data.runnumber, 0);
    delay_us(1000000);
    time = 0;
    Motor_Enable();
    if (PlayMaze() == 0) {
        show_number((time+(SCANPERSECOND/20))/(SCANPERSECOND/10), 1);
    }
    where_am_i = 0;
    data_log_finish();
    putstr(1, 7, "Log data saved", 0);

    Motor_Speed(0, 0);
    while (kbdread() != KEY_DOWN) continue;
    Motor_Disable();
}

void Configure(void) {
	const menuitem_t menu_item[] = { 
        {"Edit path       ", &edit_path,                execute},
		{"PathLength xxxxx", &data.pathlength,			decimal},
		{"SQinit  hhhhhhhh", &data.sq_init,				hex32},
		{"RunNumber  xxxxx", &data.runnumber,			decimal},
		{"Map size   xxxxx", &data.map_size,			decimal},
		{"StartPoint xxxxx", &data.green_cell_nr,		decimal},
		{"End Point  xxxxx", &data.red_cell_nr,			decimal},
		{"Save&Exit       ", 0,							none},
		{"Threshold  xxxxx", &data.threshold,			decimal},
		{"IR level   xxxxx", &data.ir_led_level,		decimal},
		{"On Way     xxxxx", &data.on_way,				decimal},
		{"MAX motor  xxxxx", &data.maxmotor,			decimal},
		{"MAX speed  xxxxx", &data.maxspeed,			decimal},
		{"MIN speed  xxxxx", &data.minspeed,			decimal},
		{"Turn speed xxxxx", &data.turnspeed,			decimal},
		{"Accelerat  xxxxx", &data.acceleration,		decimal},
		{"K*error    xxxxx", &data.k_error,				decimal},
		{"K*de/dt    xxxxx", &data.k_diff,				decimal},
		{"Kintegral  xxxxx", &data.k_integral,			decimal},
		{"Left Hand  xxxxx", &data.lefthand,			decimal},
		{"Color red  xxxxx", &data.color_red_thr, 	    decimal},
		{"Color greenxxxxx", &data.color_green_thr,	    decimal},
		{"Color blue xxxxx", &data.color_blue_thr,	    decimal},
		{"Color thre xxxxx", &data.color_threshold,     decimal},
		{"Color sat  xxxxx", &data.color_saturation,    decimal},
		{"DstToleran xxxxx", &data.tolerance,			decimal},
		{"Guard Dist xxxxx", &data.guarddist,			decimal},
		{"SensorOffs xxxxx", &data.sensor_offset,		decimal},
		{"IgnoreErrorxxxxx", &data.ignore_coordinate_error, decimal},
		{"Brake Path xxxxx", &data.brake_path,			decimal},
		{"MotorKprop xxxxx", &data.motor_Kprop,			decimal},
		{"MotorKint  xxxxx", &data.motor_Kint,			decimal},
		{"TimeToRun  xxxxx", &data.timetorun,			decimal},
		{"Turn cost  xxxxx", &data.turncost,			decimal},
		{"Cross cost xxxxx", &data.crosscost,			decimal},
		{"Step cost  xxxxx", &data.stepcost,            decimal},
		{"Str Accel  xxxxx", &data.str_accel,           decimal},
		{"Cell step  xxxxx", &data.p_step,              decimal},
		{"Voltage Calxxxxx", &data.volt_calibr,			decimal},
		{"LowBat Lvl xxxxx", &data.low_battery_level, decimal},
		{"Edit path       ", &edit_path,				execute},
		{"Save&Exit       ", 0,                         none}
	};

	Motor_Speed(0, 0);
	do_menu((menuitem_t*) menu_item, ((sizeof(menu_item)/sizeof(menu_item[0]))-1));
//	data_ptr = (uint32_t*) &data;

	data.crc32 = calc_crc32((uint8_t*)&data, sizeof(data)-4);
	spi_write_eeprom(EEPROM_COPY_ADDRESS, (uint8_t*) &data, sizeof(data));  // сначала записываем резерв
	spi_write_eeprom(EEPROM_CONFIG_ADDRESS, (uint8_t*) &data, sizeof(data));	// теперь, нормальный вариант.
	Reflectance_Init_with_Timer(data.threshold);
	ADC0_InitSWTriggerCh21();
}

void TestReflect(void) {
	uint32_t photo_sensor, photo_mask, first_sensor, last_sensor, i;
	uint8_t photo_sensor_copy[8];
	while (kbdread() != KEY_DOWN) {
		if (photo_data_ready ) {
			photo_data_ready  = 0;
			photo_sensor = current_sensor;
			photo_mask = photo_sensor;
			first_sensor = 256;
			last_sensor = 256;
			for (i=0; i<8; i++) {
				photo_sensor_copy[i] = photo_array[i];
				if (photo_mask & 0x01u) {
					last_sensor = i;
					if (first_sensor == 256) first_sensor = i;
				}
				photo_mask >>= 1;
			}
			if (first_sensor == 256) {first_sensor = 3; last_sensor = 4; }
			show_hystogram(photo_sensor, photo_sensor_copy, (data.threshold), line_error(first_sensor, last_sensor));
		}
	}
}


void TestColor(void) {
	uint16_t color_array[4] = {0, 0, 0, 0};
	unsigned int i;
	t_color field_test_color;

	ColorSensorTestHSI(color_array, 1);
	while (kbdread() != KEY_DOWN) {
	    field_test_color = check_color();
//        copy_data_dma((uint8_t *)color_sensors, (uint8_t *)color_array, sizeof(color_array));
		switch (field_test_color) {
			case red:       LaunchPad_Output(RED); break;
			case green:     LaunchPad_Output(GREEN); break;
			case blue:      LaunchPad_Output(BLUE); break;
			case yellow:    LaunchPad_Output(RED  | GREEN); break;
			case cyan: 		LaunchPad_Output(BLUE | GREEN); break;
			case magenta:   LaunchPad_Output(BLUE | RED); break;
			case white:     LaunchPad_Output(BLUE | RED | GREEN); break;
			case black:     LaunchPad_Output(0x00); break;
		}
		for (i=0; i<4; i++) {
			color_array[i] = color_sensors[i];
		}
//		while(dma_copy_busy) WaitForInterrupt();
		ColorSensorTestHSI(color_array, 0);
	}
	LaunchPad_Output(0);
}

void TestBump() {
	uint8_t bump_sensor=0;
	int impact_angle;
	unsigned int i, mask;
	char clipboard[12];
	squareXY( 0,  0, 127, 63, 0);
	squareXY(12, 23, 114, 40, 1);
	putstr(0, 1, "Bump sensor test\0", 0);
	putstr(1, 6, "Press Key Down\0", 0);
	putstr(3, 7, "to finish\0", 0);
	update_display();
	while (kbdread() != KEY_DOWN) {
		bump_sensor = Bump_Read();
		impact_angle = Bump_angle(bump_sensor);
		num2str(impact_angle, clipboard);
		putstr(0, 0, "      ", 0);
		putstr(0, 0, clipboard, 0);
		mask = 1 << 0;
		for (i = 0; i < 6; i++) {
			show_arrow((6-i)*17-4, 3, (bump_sensor & mask) ? back : straight);
			mask <<= 1;
		}
		delay_us(20000);
	}
}

void TestMotor(void) {
	unsigned int step = 0, update = 0;
	squareXY( 0,  0, 127, 63, 0);
	putstr(3, 1, "Motor test\0", 0);
	putstr(1, 6, "Press Key Down\0", 0);
	putstr(3, 7, "to finish\0", 0);
	squareXY(55, 23, 72, 40, 1);
	update_display();
	Motor_Enable();
	time_delay = SCANPERSECOND*750/1000;
	while(kbdread() != KEY_DOWN) {
		switch (step) {
			case 0:
			case 1:
				if (update) update_display();
				Motor_Speed(0, 0); 
				break;

			case 2:
				if (update) show_arrow(56, 3, right);
				Motor_Speed(data.turnspeed, -data.turnspeed); 
				break;

			case 3:
				if (update) show_arrow(56, 3, left);
				Motor_Speed(-data.turnspeed, data.turnspeed); 
				break;

			case 4:
				if (update) show_arrow(56, 3, straight);
				Motor_Speed(data.turnspeed, data.turnspeed); 
				break;

			case 5:
				if (update) show_arrow(56, 3, back);
				Motor_Speed(-data.turnspeed, -data.turnspeed); 
				break;
		}
		update = 0;
		if (!time_delay) {
			if (++step > 5) step = 0;
			time_delay = SCANPERSECOND*750/1000;
			update = 1;
		}
	}
	Motor_Speed(0, 0);
	Motor_Disable();
}


void TestTachom(void)				{
	unsigned int keyb, str_len, i;
	char clipboard[12], 
		left_str[17] = "Left ...........\0", 
		right_str[17]= "Right...........\0",
		dist_str[17] = "Dist ...........\0";
	squareXY(0, 0, 127, 63, 0);  // очистка экрана
	putstr(0, 0, "Tachometer test \0", 0);
	update_display();
	
	while (1) {
		putstr(0, 2, left_str, 0);
		putstr(0, 3, right_str, 0);
		putstr(0, 4, dist_str, 0);
		keyb  = kbdread();
		switch (keyb) {
			case KEY_UP:
				RightSteps = 0;
				LeftSteps  = 0;
				break;
			
			case KEY_DOWN:
				return;
		}

		str_len = num2str(LeftSteps, clipboard);
		for (i = 15; str_len--; i--) {
			left_str[i] = clipboard[str_len];
		}
		while (i > 4) left_str[i--] = ' ';

		str_len = num2str(RightSteps, clipboard);
		for (i = 15; str_len--; i--) {
			right_str[i] = clipboard[str_len];
		}
		while (i > 4) right_str[i--] = ' ';

		str_len = num2str((RightSteps+LeftSteps)*11/36, clipboard);
		for (i = 15; str_len--; i--) {
			dist_str[i] = clipboard[str_len];
		}
		while (i > 4) dist_str[i--] = ' ';
		delay_us(20000);
	}					
}


void Explore_Maze(void)	{
    volatile unsigned int record_count = 0;

	show_number(data.runnumber, 0);
	FRAM_log_Start(0x0004);
	frames_to_go = FRAM_SIZE/sizeof(data_buffer_t);
	delay_us(1000000);
	time = 0;
	Motor_Enable();
	if (solveMaze(1) == 0) {
		show_number((time+(SCANPERSECOND/20))/(SCANPERSECOND/10), 1);
	} 
    where_am_i = 0;
    data_log_finish();
    FRAM_log_Stop();
    record_count = FRAM_SIZE/sizeof(data_buffer_t) - frames_to_go;
    FRAM_log_Start(0x0000);
    FRAM_log_write((uint8_t*)&record_count, ((void* )0 ), sizeof(record_count));
    FRAM_log_Stop();
    frames_to_go = 0;
	Motor_Speed(0, 0);

	data.crc32 = calc_crc32((uint8_t*)&data, sizeof(data)-4);
	spi_write_eeprom(EEPROM_COPY_ADDRESS, (uint8_t*) &data, sizeof(data));  // сначала записываем резерв
	spi_write_eeprom(EEPROM_CONFIG_ADDRESS, (uint8_t*) &data, sizeof(data));	// теперь, нормальный вариант.
    while (kbdread() != KEY_DOWN) continue;
	Motor_Disable();
	Search_Short_Way_with_turns();
}

void DrawMap(void) {
//	unsigned int i;
//	data_ptr = (uint32_t*) &map;
//	src_ptr  = (uint32_t*) ROM_map_addr; //0x30000;
//	for (i=0; i<sizeof(map)/4; i++) {
//		*data_ptr++ = *src_ptr++;
//	}
	spi_read_eeprom(ROM_map_addr, (uint8_t *)&map, sizeof(map));
	Draw_Map();
	while(kbdread() != KEY_DOWN) continue;
}

void SearchShortWay() {
    Draw_Map();
    Search_Short_Way_with_turns();
    Draw_Map();

//    UART0_OutString("Calculation time: ");
//    UART0_OutUDec(calculation_time);
//    UART0_OutString("/3 us\r\n");

    while(kbdread() != KEY_DOWN) continue;
}

void ShowBattery(void) {
	squareXY(0, 0, 127, 63, 0); // clear screen
	update_display();
	while(kbdread() != KEY_DOWN) {
		if (LPF_battery.data_ready) {
			LPF_battery.data_ready = 0;
			show_number(get_battery_voltage(), 1);
			if (ADC14->IFGR1 & ADC14_IFGR1_LOIFG) {
				putstr(3, 7, "Battery Low", 0);
			}
		}
//		Clock_Delay1ms(50);
		delay_us(50000);
	}
	ADC14->CLRIFGR1 = ADC14_CLRIFGR1_CLRLOIFG;
}

void test_sq_maze(void) {
    solve_sq_maze();
    while(kbdread() != KEY_DOWN) continue;
}

void HardFault_Handler(void) {
	Motor_Disable();
	LaunchPad_LED(1);
	LaunchPad_Output(7);
	while(1) continue;
}
