/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"  // Device header
#include <stdbool.h>

bool stationary = false;
#define RED_LED	12 	// PortC Pin 12
#define GREEN_LED1	11	// PortC Pin 11
#define GREEN_LED2	10	// PortC Pin 10
#define GREEN_LED3	6 	// PortC Pin 6
#define GREEN_LED4	5 	// PortC Pin 5
#define GREEN_LED5	4 	// PortC Pin 4
#define GREEN_LED6	3 	// PortC Pin 3
#define GREEN_LED7	0 	// PortC Pin 0
#define GREEN_LED8	7 	// PortC Pin 7
#define BACKWARDSL 0 // PortB Pin 1 Backwards Motor Right
#define BACKWARDSR 1 // PortB Pin 0 Backwards Motor Left
#define MASK(x) (1 << (x))

/*----------------------------------------------------------------------------
 * UART THINGS
 *---------------------------------------------------------------------------*/

typedef struct
{
	bool playEndingMusic;
	bool backwardL;
	bool backwardR;
	int8_t leftMotorStrength;
	int8_t rightMotorStrength;
} myDataPkt;

osMessageQueueId_t commandQueue;

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

uint8_t command;

// 8N1
/* Init UART2 */
void initUART2(uint32_t baud_rate)
{
    uint32_t divisor, bus_clock;

    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

		// No need for transmit
    //PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    //PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);

    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

    bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
    divisor = bus_clock / (baud_rate * 16);
    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    UART2->BDL = UART_BDL_SBR(divisor);

    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;

    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_RIE_MASK));
		
		NVIC_SetPriority(UART2_IRQn, 2);
		NVIC_ClearPendingIRQ(UART2_IRQn);
		NVIC_EnableIRQ(UART2_IRQn);
}

void UART2_IRQHandler() {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		command = (UART2->D);
	}	
}

/* UART2 Transmit Poll
void UART2_Transmit_Poll(uint8_t data)
{
    while (!(UART2->S1 & UART_S1_TDRE_MASK));
    UART2->D = data;
}
*/

/* UART2 Receive Poll 
uint8_t UART2_Receive_Poll(void)
{
    while (!(UART2->S1 & UART_S1_RDRF_MASK));
    return (UART2->D);
} */

/*----------------------------------------------------------------------------
 * LED THINGS
 *---------------------------------------------------------------------------*/

typedef enum led_colors {
	red_led = RED_LED,
	green_led1 = GREEN_LED1,
	green_led2 = GREEN_LED2,
	green_led3 = GREEN_LED3,
	green_led4 = GREEN_LED4,
	green_led5 = GREEN_LED5,
	green_led6 = GREEN_LED6,
	green_led7 = GREEN_LED7,
	green_led8 = GREEN_LED8,
}led_colors_t;

typedef enum led_switch {
	led_on = 1,
	led_off = 0,
}led_switch_t;

void offRGB(void) {
	PTC->PCOR = MASK(GREEN_LED1) |
							MASK(GREEN_LED2) |
							MASK(GREEN_LED3) |
							MASK(GREEN_LED4) |
							MASK(GREEN_LED5) |
							MASK(GREEN_LED6) |
							MASK(GREEN_LED7) |
							MASK(GREEN_LED8);
	PTC->PCOR = MASK(RED_LED);
}
void offGreenLEDs(void) {
	PTC->PCOR = MASK(GREEN_LED1) |
							MASK(GREEN_LED2) |
							MASK(GREEN_LED3) |
							MASK(GREEN_LED4) |
							MASK(GREEN_LED5) |
							MASK(GREEN_LED6) |
							MASK(GREEN_LED7) |
							MASK(GREEN_LED8);
}

void ledControl(led_colors_t colour, led_switch_t switchOn) {
	if (switchOn) {
		switch(colour)
		{
			case RED_LED: 
				PTC->PSOR |= MASK(RED_LED);
				break;
			case GREEN_LED1:
				PTC->PSOR |= MASK(GREEN_LED1);
				break;
			case GREEN_LED2:
				PTC->PSOR |= MASK(GREEN_LED2);
				break;
			case GREEN_LED3:
				PTC->PSOR |= MASK(GREEN_LED3);
				break;
			case GREEN_LED4:
				PTC->PSOR |= MASK(GREEN_LED4);
				break;
			case GREEN_LED5:
				PTC->PSOR |= MASK(GREEN_LED5);
				break;
			case GREEN_LED6:
				PTC->PSOR |= MASK(GREEN_LED6);
				break;
			case GREEN_LED7:
				PTC->PSOR |= MASK(GREEN_LED7);
				break;
			case GREEN_LED8:
				PTC->PSOR |= MASK(GREEN_LED8);
				break;
			default:
				offRGB();
		}	
	} else {
			//offRGB();
				switch(colour)
		{
			case RED_LED: 
				PTC->PCOR |= MASK(RED_LED);
				break;
			case GREEN_LED1:
				PTC->PCOR |= MASK(GREEN_LED1);
				break;
			case GREEN_LED2:
				PTC->PCOR |= MASK(GREEN_LED2);
				break;
			case GREEN_LED3:
				PTC->PCOR |= MASK(GREEN_LED3);
				break;
			case GREEN_LED4:
				PTC->PCOR |= MASK(GREEN_LED4);
				break;
			case GREEN_LED5:
				PTC->PCOR |= MASK(GREEN_LED5);
				break;
			case GREEN_LED6:
				PTC->PCOR |= MASK(GREEN_LED6);
				break;
			case GREEN_LED7:
				PTC->PCOR |= MASK(GREEN_LED7);
				break;
			case GREEN_LED8:
				PTC->PCOR |= MASK(GREEN_LED8);
				break;
			default:
				offRGB();
		}	
	}
}

void InitGPIO(void) {
	// Enable Clock to PORTB & PORTC
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK));
	
	// Configure MUX settings to make all GREEN_LEDx pins GPIO
	PORTC->PCR[GREEN_LED1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED1] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED2] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED3] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED4] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED5] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED6] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED7] |= PORT_PCR_MUX(1);
	PORTC->PCR[GREEN_LED8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED8] |= PORT_PCR_MUX(1);
	PORTC->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	// Configure MUX settings to make motor backwards pin GPIO
	PORTB->PCR[BACKWARDSL] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BACKWARDSL] |= PORT_PCR_MUX(1);
	PORTB->PCR[BACKWARDSR] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BACKWARDSR] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortC
	PTC->PDDR |= (MASK(GREEN_LED1) |
								MASK(GREEN_LED2) |
								MASK(GREEN_LED3) |
								MASK(GREEN_LED4) |
								MASK(GREEN_LED5) |
								MASK(GREEN_LED6) |
								MASK(GREEN_LED7) |
								MASK(GREEN_LED8) |
								MASK(RED_LED));
								
	// Set Data Direction Registers for PortC
	PTB->PDDR |= MASK(BACKWARDSL);
	PTB->PDDR |= MASK(BACKWARDSR);
}
 
#define PTB0_Pin 0 // Speaker 				(TPM1_CH0)
#define PTB2_Pin 2 // Left motor FW		(TPM2_CH0) Blue wire
#define PTB3_Pin 3 // Right motor FW 	(TPM2_CH1) Purple wire

int leftMotorSpeed = 0; 			// Out of 8
int rightMotorSpeed = 0; 			// Out of 8
bool movingBackwards = false;		// False: forwards and True: backwards

void initMotorPWM() { // TPM2
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK; // Clear register
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3); // pg163 MUX
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK; // Clear register
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3); // pg163 MUX
	
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	// System Options Register, clear and set TPMSRC to 1, pg195
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // 48MHz?
	
	TPM2->MOD = 3; // pg 554
	TPM2_C0V = leftMotorSpeed;
	TPM2_C1V = rightMotorSpeed;
	
	// TPM status control
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Channel status and control pg555, (3 x TPM has 6 channels each)
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear bits
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear bits
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

}
/*
void initPWM(uint16_t mod_value) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK; // Clear register
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3); // pg163 MUX
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK; // Clear register
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3); // pg163 MUX
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	// System Options Register, clear and set TPMSRC to 1, pg195
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // 48MHz?
		
	TPM1_C1V = backwardMotorSpeed;
	TPM1->MOD = 442; // pg 554 7500 for 50Hz, May need to edit for tone
	TPM1_C0V = TPM1->MOD / 7;
	
	// TPM status control
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Channel status and control pg555, (3 x TPM has 6 channels each)
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear bits
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}*/
#define MOD(x) (37500/x)

#define PTC4_Pin 4 //TPM0_CH3

#define C3  130
#define G3  196
#define F3  175
#define Eb3 156

#define Fs3 185
#define Ab3 208
#define A3  220
#define Bb3 233
#define B3  247

#define C4  261
#define D4  294
#define Eb4 311
#define E4  330
#define F4  349
#define Fs4 370
#define G4  392
#define Ab4 415
#define A4  440
#define As4 467
#define B4  494
#define C5  523
#define Cs5 554
#define D5  550
#define Eb5 622
#define REST4 0

void initPWM(uint16_t mod_value) {
 
 //Enables the clock gate for Port C
 SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
 
 PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK; //Clear bit 10 to 8
 PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(4); //Enable Timer function of pin
 
 //Enables clock gate for TPM1, page 208
 SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; 
 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clear bit 25 to 24, datasheet page 195
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //Set 01 to bit 25 to 24, MCGFLLCLK clock or MCGPLLCLK/2 is used as clock source for TPM counter clock
 
 TPM0->MOD = 7500; //Set Modulo value = 4 800 000 / 128 = 375 000 / 7500 = 50 Hz
 
 //Datasheet 553, LPTPM means low power timer/pulse width modulator module
 TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //Clears bit 4 to 0, 2 to 0 for PS, 4 to 3 for CMOD
 TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // LPTPM counter clock mode is selected as 01 (LPTPM counter increments on every LPTPM counter clock), Prescaler 128 
 TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clears CPWMS (Centre-aligned PWM select). Aka mode = 0 which means LPTPM counts up
 
 //Datasheet 555
 TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); //Clears bit 5 to 2, disabling channel mode 
 TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // CPWMS = 0, ELSnB:ELSnA = 0b10, MSnB:MSnA = 0b10, this means Mode = Edge-aligned PWM, Config = High-true pulses (clear Output on match, setOutput on reload)
}
#define MOD_music(x) (75000/x)

int finish_run[] = {
    C5,Eb4,D4,Eb4, C4,Eb4,D4,Eb4, C5,Eb4,D4,Eb4, C4,Eb4,D4,Eb4,
  	Ab4,F4,E4,F4, C4,F4,E4,F4, Ab4,F4,E4,F4, C4,F4,E4,F4, 
	  B4,F4,Eb4,F4, D4,F4,Eb4,F4, B4,F4,Eb4,F4, D4,F4,Eb4,F4,
	  C5,G4,F4,G4, Eb4,G4,F4,G4, C5,G4,F4,G4, Eb4,G4,F4,G4,
	  Eb5,Ab4,G4,Ab4, Eb4,Ab4,G4,Ab4, Eb5,Ab4,G4,Ab4, Eb4,Ab4,G4,Ab4,
	  D5,Fs4,E4,Fs4, D4,Fs4,E4,Fs4, D5,Fs4,E4,Fs4, D4,Fs4,E4,Fs4,
	  D5,G4,Fs4,G4, D4,G4,Fs4,G4, D5,G4,Fs4,G4, D4,G4,Fs4,G4,
	  C5,E4,D4,E4, C4,E4,D4,E4, C5,E4,D4,E4, C4,E4,D4,E4,
	  C5,F4,E4,F4, C4,F4,E4,F4, C5,F4,E4,F4, C4,F4,E4,F4,
		As4,G4,F4,G4, Eb4,G4,F4,G4, As4,G4,F4,G4, Eb4,G4,F4,G4,
		Ab4,G4,F4,G4, Eb4,G4,F4,G4, Ab4,G4,F4,G4, Eb4,G4,F4,G4,
		Ab4,D4,C4,D4, Bb3,D4,C4,D4, Ab4,D4,C4,D4, Bb3,D4,C4,D4,
		G4,Bb3,Ab3,Bb3, Eb4,Bb3,Ab3,Bb3, G4,Bb3,Ab3,Bb3, Eb4,Bb3,Ab3,Bb3,
		F4,C4,Bb3,C4, A3,C4,Bb3,C4, F4,C4,Bb3,C4, A3,C4,Bb3,C4,
		F4,D4,C4,D4, B3,D4,C4,D4, F4,D4,C4,D4, B3,D4,C4,D4,
		Eb4,C4,B3,C4, G3,C4,B3,C4, Eb4,C4,B3,C4, G3,C4,B3,C4,
		F3,Eb4,D4,Eb4, F4,Eb4,D4,Eb4, F3,Eb4,D4,Eb4, F4,Eb4,D4,Eb4,
		Fs3,C4,B3,C4, Eb4,C4,B3,C4, Fs3,C4,B3,C4, Eb4,C4,B3,C4,
		Eb4,C4,B3,C4, G3,C4,B3,C4, Eb4,C4,B3,C4, G3,C4,B3,C4, 
		Fs4,C4,B3,C4, A3,C4,B3,C4, Fs4,C4,B3,C4, A3,C4,B3,C4, 
		G4,C4,B3,C4, D4,C4,B3,C4, G4,C4,B3,C4, D4,C4,B3,C4, 
		Ab4,C4,B3,C4, D4,C4,B3,C4, Ab4,C4,B3,C4, D4,C4,B3,C4, 
};





//int finish_run[] = {
	//B4,B4,B4,B4, B4,REST4,B4,B4, B4,B4,B4,B4, B4,REST4,E4,E4, E4,E4,E4,E4, E4,REST4,D4,D4, D4,D4,D4,D4, D4,REST4,A4,A4,
	//B4,B4,B4,B4, B4,REST4,B4,B4, B4,B4,B4,B4, B4,REST4,E4,E4, B4,B4,B4,B4, B4,REST4,E4,E4
//};

void play_finish(void *argument){
	osDelay(2000);
    int notes_num = sizeof(finish_run)/ sizeof(finish_run[0]);
 int beats_per_min = 500;
	//400
 
 int one_beat = 50000 / beats_per_min; //60000 ms = 60 seconds
 
	for(;;) {
		for(int i = 0; i < notes_num; i++) {
 
			int curr_musical_note = finish_run[i] - 18;
  
			int period = MOD_music(curr_musical_note);
  
			TPM0->MOD = period;
			TPM0_C3V = period / 6; 
  
			osDelay(one_beat); //all equal in length
		}
 }
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

osMutexId_t myMutex;
osMutexId_t greenLedMutex;

void parseCommand(void *argument) {
	for (;;) {
		//uint8_t command = UART2_Receive_Poll();
		myDataPkt dataPkt;
		dataPkt.leftMotorStrength = (command & 0b00011000) >> 3;
		dataPkt.rightMotorStrength = (command & 0b00000011);
		dataPkt.backwardL = (command & 0b00000100) >> 2;
		dataPkt.backwardR = (command & 0b00100000) >> 5;
		if (dataPkt.leftMotorStrength == 0 && dataPkt.rightMotorStrength == 0) {
			stationary = true;
		} else {
			stationary = false;
		}
		dataPkt.playEndingMusic = (command & 0b10000000) >> 7;
		
		osMessageQueuePut(commandQueue, &dataPkt, NULL, 0);
		
		// use the following command
		//osMessageQueueGet(commandQueue, &myRxData, NULL, osWaitForever); // myRxData is a myDataPkt defined in the receiving thread
	}
}

int polarity = 1;

void motor_thread(void *argument) {
	for (;;) {
		myDataPkt rxData;
		osMessageQueueGet(commandQueue, &rxData, NULL, osWaitForever);
		
		if (!rxData.leftMotorStrength && !rxData.rightMotorStrength) {
			stationary = 1;
		} else {
			stationary = 0;
		}
		
		if (!rxData.backwardL) {
				PTB->PCOR |= MASK(BACKWARDSL);
				TPM2_C0V = rxData.leftMotorStrength;
		} else {
				PTB->PSOR |= MASK(BACKWARDSL);
				TPM2_C0V = 3 - rxData.leftMotorStrength;
		}
		
		
		if (!rxData.backwardR) {
				PTB->PCOR |= MASK(BACKWARDSR);
				TPM2_C1V = rxData.rightMotorStrength;
		} else {
				PTB->PSOR |= MASK(BACKWARDSR);
				TPM2_C1V = 3 - rxData.rightMotorStrength;
		}
			
	}
}
/*
void led_green_thread1 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led1, led_on); // led_on
			osDelay(1000);
			ledControl(green_led1, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(7000);
		} else {
			ledControl(green_led1, led_on); // led_on
			ledControl(green_led2, led_on); // led_on
			ledControl(green_led3, led_on); // led_on
			ledControl(green_led4, led_on); // led_on
			ledControl(green_led5, led_on); // led_on
			ledControl(green_led6, led_on); // led_on
			ledControl(green_led7, led_on); // led_on
			ledControl(green_led8, led_on); // led_on
			ledControl(red_led, led_on); // led_on
			osDelay(250);
			ledControl(red_led, led_off); // led_off
			osDelay(250);
		}
	}
}
void led_green_thread2 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led2, led_on); // led_on
			osDelay(1000);
			ledControl(green_led2, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(6000);
		}
	}
}
void led_green_thread3 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led3, led_on); // led_on
			osDelay(1000);
			ledControl(green_led3, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(5000);
		}
	}
}
void led_green_thread4 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led4, led_on); // led_on
			osDelay(1000);
			ledControl(green_led4, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(4000);
		}
	}
}
void led_green_thread5 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led5, led_on); // led_on
			osDelay(1000);
			ledControl(green_led5, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(3000);
		}
	}
}
void led_green_thread6 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led6, led_on); // led_on
			osDelay(1000);
			ledControl(green_led6, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(2000);
		}
	}
}
void led_green_thread7 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led7, led_on); // led_on
			osDelay(1000);
			ledControl(green_led7, led_off); // led_off
			osMutexRelease(greenLedMutex);
			osDelay(1000);
		}
	}
}
void led_green_thread8 (void *argument) {
  for (;;) {
		if (!stationary) {
			osMutexAcquire(greenLedMutex, osWaitForever);
			ledControl(green_led8, led_on); // led_on
			osDelay(1000);
			ledControl(green_led8, led_off); // led_off
			osMutexRelease(greenLedMutex);
		}
	}
} //*/
void green_led_thread (void *argument) {
	for (;;) {
		//offGreenLEDs();
		if (!stationary) {
			ledControl(green_led1, led_on); // led_on
			osDelay(500);
			ledControl(green_led1, led_off); // led_off
			ledControl(green_led2, led_on); // led_on
			osDelay(500); 
			ledControl(green_led2, led_off); // led_off
			ledControl(green_led3, led_on); // led_on
			osDelay(500); 
			ledControl(green_led3, led_off); // led_off
			ledControl(green_led4, led_on); // led_on
			osDelay(500); 
			ledControl(green_led4, led_off); // led_off
			ledControl(green_led5, led_on); // led_on
			osDelay(500); 
			ledControl(green_led5, led_off); // led_off
			ledControl(green_led6, led_on); // led_on
			osDelay(500); 
			ledControl(green_led6, led_off); // led_off
			ledControl(green_led7, led_on); // led_on
			osDelay(500); 
			ledControl(green_led7, led_off); // led_off
			ledControl(green_led8, led_on); // led_on
			osDelay(500); 
			ledControl(green_led8, led_off); // led_off
		} else {
			ledControl(green_led1, led_on); // led_on
			ledControl(green_led2, led_on); // led_on
			ledControl(green_led3, led_on); // led_on
			ledControl(green_led4, led_on); // led_on
			ledControl(green_led5, led_on); // led_on
			ledControl(green_led6, led_on); // led_on
			ledControl(green_led7, led_on); // led_on
			ledControl(green_led8, led_on); // led_on
		}
	}
}
void led_red_thread (void *argument) {
  for (;;) {
		if (!stationary) {
			ledControl(red_led, led_on); // led_on
			osDelay(500);
			ledControl(red_led, led_off); // led_off
			osDelay(500);
		} else {
			ledControl(red_led, led_on); // led_on
			osDelay(250);
			ledControl(red_led, led_off); // led_off
			osDelay(250);
		}
	}
}
///*
const osThreadAttr_t thread_attr = {
	.priority = osPriorityNormal7
};

const osThreadAttr_t greenLed1Priority = {
	//.priority = osPriorityBelowNormal7
	.priority = osPriorityNormal7
};
const osThreadAttr_t greenLed2Priority = {
	//.priority = osPriorityBelowNormal6
	.priority = osPriorityNormal6
};
const osThreadAttr_t greenLed3Priority = {
	//.priority = osPriorityBelowNormal5
	.priority = osPriorityNormal5
};
const osThreadAttr_t greenLed4Priority = {
	//.priority = osPriorityBelowNormal4
	.priority = osPriorityNormal4
};
const osThreadAttr_t greenLed5Priority = {
	//.priority = osPriorityBelowNormal3
	.priority = osPriorityNormal3
};
const osThreadAttr_t greenLed6Priority = {
	//.priority = osPriorityBelowNormal2
	.priority = osPriorityNormal2
};
const osThreadAttr_t greenLed7Priority = {
	//.priority = osPriorityBelowNormal1
	.priority = osPriorityNormal1
};
const osThreadAttr_t greenLed8Priority = {
	//.priority = osPriorityBelowNormal
	.priority = osPriorityNormal
}; //*/
const osThreadAttr_t motorPriority = {
	.priority = osPriorityNormal7
	//.priority = osPriorityHigh
};

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	offRGB();
	initPWM(0);
	initMotorPWM();
	initUART2(BAUD_RATE);
  // ...
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	//myMutex = osMutexNew(NULL);
	//greenLedMutex = osMutexNew(NULL);
	
	// Motor threads
	osThreadNew(motor_thread, NULL, &motorPriority);
	osThreadNew(parseCommand, NULL, &motorPriority);
	
	// LED threads
	osThreadNew(green_led_thread, NULL, &motorPriority);
	osThreadNew(led_red_thread, NULL, &motorPriority);
	/*
	osThreadNew(led_green_thread1, NULL, &greenLed1Priority);
	osThreadNew(led_green_thread2, NULL, &greenLed2Priority);
	osThreadNew(led_green_thread3, NULL, &greenLed3Priority);
	osThreadNew(led_green_thread4, NULL, &greenLed4Priority);
	osThreadNew(led_green_thread5, NULL, &greenLed5Priority);
	osThreadNew(led_green_thread6, NULL, &greenLed6Priority);
	osThreadNew(led_green_thread7, NULL, &greenLed7Priority);
	osThreadNew(led_green_thread8, NULL, &greenLed8Priority);
	*/
	commandQueue = osMessageQueueNew(1, sizeof(myDataPkt), NULL);
	
	// Music threads
	//osThreadNew(play_finish, NULL, NULL);
  osKernelStart();                      // Start thread execution
	
  for (;;) {

	}
}
