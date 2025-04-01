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
#define MASK(x) (1 << (x))

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
// Enable Clock to PORTC
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);
	
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
}
 
#define PTB0_Pin 0 // Speaker 				(TPM1_CH0)
#define PTB1_Pin 1 // Backwards motor	(TPM1_CH1) Ivory wire
#define PTB2_Pin 2 // Left motor FW		(TPM2_CH0) Blue wire
#define PTB3_Pin 3 // Right motor FW 	(TPM2_CH1) Purple wire

int leftMotorSpeed = 0; 			// Out of 100
int rightMotorSpeed = 0; 			// Out of 100
int backwardMotorSpeed = 0;		// Out of ???, depends on speaker lol which is fked

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
	
	TPM2->MOD = 100; // pg 554
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

}/*
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
#define NOTE_C4  261
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_As4  467
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  550
#define REST 0

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
    NOTE_C4, NOTE_D4, NOTE_F4, NOTE_C5, NOTE_A4, REST, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_A4, NOTE_G4
		, REST, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_A4, NOTE_G4
		, REST, NOTE_A4, NOTE_D4, NOTE_F4, NOTE_G4, NOTE_F4
		, REST, NOTE_A4, NOTE_As4, NOTE_C5, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_C5
		, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_C5
		, NOTE_C5, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_C4, NOTE_F4, NOTE_F4
		, REST, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4
};

void play_finish(void *argument){
	osDelay(2000);
    int notes_num = sizeof(finish_run)/ sizeof(finish_run[0]);
 int beats_per_min = 120;
 
 int one_beat = 60000 / beats_per_min; //60000 ms = 60 seconds
 
 for(int i = 0; i < notes_num; i++)
 {
  int curr_musical_note = finish_run[i];
  
  int period = MOD_music(curr_musical_note);
  
  TPM0->MOD = period;
  TPM0_C3V = period / 6; 
  
  osDelay(one_beat); //all equal in length
 }
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

osMutexId_t myMutex;
osMutexId_t greenLedMutex;

int polarity = 1;

void left_motor_thread(void *argument) {
	for (;;) {
		leftMotorSpeed = leftMotorSpeed + polarity;
		osDelay(10);
		if (leftMotorSpeed == 100) {
			polarity = -1;
		} else if (leftMotorSpeed == 0) {
			polarity = 1;
		}	
		TPM2_C0V = leftMotorSpeed;
	}
}
void right_motor_thread(void *argument) {
	for (;;) {
		rightMotorSpeed = rightMotorSpeed + polarity;
		osDelay(10);
		if (rightMotorSpeed == 100) {
			polarity = -1;
		} else if (rightMotorSpeed == 0) {
			polarity = 1;
		}
		TPM2_C1V = rightMotorSpeed;
	}
}
void turn_motor_thread(void *argument) {
	osDelay(10000);
		TPM2_C0V = 40;
		TPM2_C1V = 40;
}
void quaver() {
	osDelay(210);
	SIM_SCGC5 &= ~SIM_SCGC5_PORTB_MASK;
	osDelay(10);
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
}
void semi_quaver() {
	osDelay(100);
	SIM_SCGC5 &= ~SIM_SCGC5_PORTB_MASK;
	osDelay(10);
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
}
void B4_sq() {
	TPM1->MOD = 759; // B4
	TPM1_C0V = 759 / 2;
	semi_quaver();
}
void B4_q() {
	TPM1->MOD = 759; // B4
	TPM1_C0V = 759 / 2;
	quaver();
}
void E5_sq() {
	TPM1->MOD = 569; // E5
	TPM1_C0V = 569 / 2;
	semi_quaver();
}
void E5_q() {
	TPM1->MOD = 569; // E5
	TPM1_C0V = 569 / 2;
	quaver();
}
void D5_sq() {
	TPM1->MOD = 638; // D5
	TPM1_C0V = 638 / 2;
	semi_quaver();
}
void D5_q() {
	TPM1->MOD = 638; // D5
	TPM1_C0V = 638 / 2;
	quaver();
}
void A4_sq() {
	TPM1->MOD = 852; // E5
	TPM1_C0V = 852 / 2;
	semi_quaver();
}
void music_thread (void *argument) {
	for (;;) {
		osDelay(1000);
		//TPM1->MOD = 1860; // B4
		//TPM1_C0V = TPM1->MOD / 2;
		//TPM1->CONTROLS[1].CnV = TPM1->MOD / 2;
		//semi_quaver();
		TPM1->MOD = 1277; // B4
		TPM1_C0V = TPM1->MOD / 2;
		quaver();
		TPM1->MOD = 1136; // B4
		TPM1_C0V = TPM1->MOD / 2;
		quaver();
		TPM1->MOD = 957; // B4
		TPM1_C0V = TPM1->MOD / 2;
		quaver();
		TPM1->MOD = 638; // B4
		TPM1_C0V = TPM1->MOD / 2;
		quaver();
		TPM1->MOD = 759; // B4
		TPM1_C0V = TPM1->MOD / 2;
		quaver();
		/*
		// Bar1
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_q();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		E5_sq();
		E5_sq();
		// Bar2
		E5_sq();
		E5_sq();
		E5_sq();
		E5_sq();
		E5_q();
		D5_sq();
		D5_sq();
		D5_sq();
		D5_sq();
		D5_sq();
		D5_sq();
		D5_q();
		A4_sq();
		// Bar3
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_q();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		E5_sq();
		E5_sq();
		// Bar4
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_q();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		B4_sq();
		E5_sq();
		E5_sq();
		*/
	}
}

void led_red_thread (void *argument) {
  for (;;) {
		if (!stationary) {
			ledControl(red_led, led_on); // led_on
			osDelay(500);
			ledControl(red_led, led_off); // led_off
			osDelay(500);
		}
	}
}
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
}

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
};
const osThreadAttr_t leftMotorPriority = {
	.priority = osPriorityHigh
};
const osThreadAttr_t rightMotorPriority = {
	.priority = osPriorityHigh
};
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	offRGB();
	initPWM(0);
	initMotorPWM();
  // ...
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	myMutex = osMutexNew(NULL);
	greenLedMutex = osMutexNew(NULL);
	
	// Motor threads
	//osThreadNew(left_motor_thread, NULL, &leftMotorPriority);
	//osThreadNew(right_motor_thread, NULL, &rightMotorPriority);
	//osThreadNew(turn_motor_thread, NULL, &rightMotorPriority);
	
	// LED threads
	//osThreadNew(led_green_thread1, NULL, &greenLed1Priority);
	//osThreadNew(led_green_thread2, NULL, &greenLed2Priority);
	//osThreadNew(led_green_thread3, NULL, &greenLed3Priority);
	//osThreadNew(led_green_thread4, NULL, &greenLed4Priority);
	//osThreadNew(led_green_thread5, NULL, &greenLed5Priority);
	//osThreadNew(led_green_thread6, NULL, &greenLed6Priority);
	//osThreadNew(led_green_thread7, NULL, &greenLed7Priority);
	//osThreadNew(led_green_thread8, NULL, &greenLed8Priority);
	//osThreadNew(led_red_thread, NULL, NULL);
	
	// Music threads
	osThreadNew(play_finish, NULL, NULL);
  osKernelStart();                      // Start thread execution
	
  for (;;) {}
}
