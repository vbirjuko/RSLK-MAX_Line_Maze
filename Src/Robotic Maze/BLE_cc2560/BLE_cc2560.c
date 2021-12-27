#include <stdint.h>

#include "AP.h"
//#include "../inc/Clock.h"
//#include "../inc/CortexM.h"
#include "LaunchPad.h"
#include "main.h"

char        password[8] = "\0", *pass_ptr;
unsigned int Auth_status = 0;

void ReadMaxSpeed(void){ // called on a SNP Characteristic Read Indication for characteristic ByteData
    static int LedState = 0;
    LaunchPad_LED(LedState ^= 0x01);    // toggle LED
}

void WriteMaxSpeed(void){ // called on a SNP Characteristic Read Indication for characteristic ByteData
    static int LedState = 0;
    LaunchPad_Output(LedState ^= GREEN);    // toggle LED
}



void CheckPassword() {
    unsigned int pass = 0, i;
    const char mypassword[8] = "PASSWORD", *my_ptr;
    uint8_t NPI_Disconnect[] = {
          SOF,
          0x03, 0x00,   // length
          0x55, 0x46,   //cmd0, cmd1
          0xFF, 0xFF,   // Terminate all connections
          0x00,         // Default: gracefully disconnect by sending a termination over-the-air
          0xEE          // FSC
    };

    pass_ptr = password;
    my_ptr   = &mypassword[7];
    for (i = 0; i < sizeof(mypassword); i++) {
        if (*my_ptr-- != *pass_ptr++) {pass = 1; break; }
    }
    if (pass) {
        LaunchPad_Output(RED);
        AP_SendMessage(NPI_Disconnect);
        pass = 0;
        Auth_status = 0;
    } else {
        LaunchPad_Output(GREEN);
        Auth_status = 1;
    }
}

void ClearAuth(void) {
    Auth_status = 0;
}


void BLE_Init(void){
 // write this as part of Lab 19

    AP_Init();
    AP_GetStatus();  // optional
    AP_GetVersion(); // optional
    AP_AddService(0xFFF0);
    //------------------------
    AP_AddCharacteristic(0xFFF1,               8,&password,0x02,0x08,"Password",0,&CheckPassword);
    AP_AddCharacteristic(0xFFF2,sizeof(data.maxspeed),&data.maxspeed,	0x03,0x0A,"MaxSpeed",&ReadMaxSpeed,&WriteMaxSpeed);
    AP_AddCharacteristic(0xFFF3,sizeof(data.k_error),	&data.k_error,	0x03,0x0A,"K Error",&ReadMaxSpeed,&WriteMaxSpeed);
    AP_AddCharacteristic(0xFFF4,sizeof(data.k_diff),	&data.k_diff	,	0x03,0x0A,"K Diff",&ReadMaxSpeed,&WriteMaxSpeed);
    AP_AddCharacteristic(0xFFF5,sizeof(data.div_sigma),&data.div_sigma,0x03,0x0A,"K Integr",&ReadMaxSpeed,&WriteMaxSpeed);
    AP_AddCharacteristic(0xFFF6,sizeof(data.minspeed),&data.minspeed,	0x03,0x0A,"MinSpeed",&ReadMaxSpeed,&WriteMaxSpeed);
    AP_AddCharacteristic(0xFFF7,sizeof(data.turnspeed),&data.turnspeed,0x03,0x0A,"TurnSpeed",&ReadMaxSpeed,&WriteMaxSpeed);
/*  //------------------------
    ByteData = LaunchPad_Input(); // read only parameter (get from switches)
    AP_AddCharacteristic(0xFFF2,2,&HalfWordData,0x01,0x02,"HalfWordData",&ReadHalfWordData,0);
	//------------------------
    WordData = 0;   // write only parameter (sent to LED)
    AP_AddCharacteristic(0xFFF8,4,&WordData,0x02,0x08,"WordData",0,&WriteWordData);
  //------------------------
    Switch1 = 0;
    AP_AddNotifyCharacteristic(0xFFF4,2,&Switch1,"Button 1",&Button1);
    //------------------------
    Switch2 = 0x00000000;
    AP_AddNotifyCharacteristic(0xFFF5,4,&Switch2,"Button 2",&Button2);
*/    //------------------------
    AP_RegisterService();
    AP_StartAdvertisementJacki();
    AP_GetStatus(); // optional

}
/*
void main(void){
  DisableInterrupts();
	// write this as part of Lab 19
  Clock_Init48MHz();
  UART0_Init();
  LaunchPad_Init();  // input from switches, output to LEDs on LaunchPad
  EnableInterrupts();
  UART0_OutString("\n\rApplication Processor - MSP432-CC2650\n\r");
  EnableInterrupts();
  LaunchPad_LED(0);
  BLE_Init();
  
  while(1){
  // write this as part of Lab 19
      AP_BackgroundProcess();  // handle incoming SNP frames

  }
}
*/
