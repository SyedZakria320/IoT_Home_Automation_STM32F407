//		===========[ IOT HOME AUTOMATION PROJECT	]==========	//
//		Syed Zakriya 	[2019-EE-320]
// 		[ Section-B ]
// 		University of Engineering and Technology Lahore (New Campus)

// Header Files of Project Code contains HAL Driver and I2C Driver of LCD

#include "stm32f4xx_hal.h"
#include "STM32F407_I2C_LCD16x02_Driver.h"

void Relay_Wifi_Init();		// Function for Initializing Relay and Wifi Module

// Bit Masks for Input Pins of STM uC (PD1, PD2)

int Input_1 = 0x1<<1;		// 0b10		Wifi Module (GPIO_0) 	STM uC (PD1)
int Input_2 = 0x1<<2;		// 0b100	Wifi Module (GPIO_2)	STM uC (PD2)

// Bit Masks for Output Pins of STM uC (PE1, PE2, PE3, PE4)

int Output_1 = 0x1<<1;		// 0b10 	Relay Module GPIOE 		STM uC (PE1)
int Output_2 = 0x1<<2;		// 0b100	Relay Module GPIOE 		STM uC (PE2)
int Output_3 = 0x1<<3;		// 0b1000	Relay Module GPIOE 		STM uC (PE3)
int Output_4 = 0x1<<4;		// 0b10000	Relay Module GPIOE 		STM uC (PE4)


void On_Load(int);			// Function for Turning Loads ON anf OFF
void Relay_Test();			// Function for Testing Relays by Pressing PA0 button on STM uC

int main()
{
	HAL_Init();				// Initializing HAL Driver
	LCD_Init();				// Initializing LCD through HAL Library
	Relay_Wifi_Init();		// Initializing Relay and Wifi Module GPIO's through Function
	
	// Startup of IOT HOME AUTOMATION PROJECT

	LCD_Send_String("    IOT Home    ");
	HAL_Delay(1000);
	LCD_Send_Cmd(LCD_SET_ROW2_COL1);
	LCD_Send_String("==[Automation]==");
	HAL_Delay(1000);
	
	while (1)
	{
		Relay_Test();					// Check Working of Relays
		if (GPIOD->IDR&Input_1)			// Check Input_1 Pin PD1 logic 1
		{
			if (GPIOD->IDR&Input_2)		// Check Input_2 Pin PD2 logic 1
			{
				On_Load(4);				// Turn ON load 4
			}
			else		// Input_2 Pin PD2 logic 0
			{
				On_Load(3);				// Turn ON load 3
			}
		}
		else			// Input_1 Pin PD1 logic 0
		{
			if (GPIOD->IDR&Input_2)			// Check Input_2 Pin PD2 logic 1
			{
				On_Load(2);				// Turn ON load 2
			}
			else		// Input_2 Pin PD2 logic 2
			{
				On_Load(1);				// Turn ON Relay Module Reset State
			}
		}
	}
}

// Function for Initializing Relay and Wifi Module

void Relay_Wifi_Init()
{
	RCC->AHB1ENR |= 0x19;		// Enable GPIO Port A, Port D and Port E
	
	int pin = 0;
	
	// PA0 button as Input mode
	
	GPIOA->MODER &= ~(0x3<<pin*2);
	
	
	// GPIOD Pins PD1-PD2 Input Mode Set for Wifi Module
	
	for (pin=1; pin<3; pin++)
	{
		GPIOD->MODER &= ~(0x3<<pin*2);		// Input mode MODER [00]
	    GPIOD->MODER |=  (0x0<<pin*2);
		GPIOD->PUPDR &= ~(0x3<<pin*2);		// Pull Down PUPDR [10]
		GPIOD->PUPDR |= (0x2<<pin*2);
	}
	
	// GPIOE Pins PE1-PE4 Output Mode Set for Relay Module
	
	for (pin=1; pin<5; pin++)
	{
		GPIOE->MODER &= ~(0x3<<pin*2);		// Output mode MODER [01]
		GPIOE->MODER |=  (0x1<<pin*2);
		GPIOE->OTYPER &= ~(0x1<<pin);		// Open-Drain OTYPER [1]
		GPIOE->OTYPER |=  (0x1<<pin);
		GPIOE->ODR |= (0x1<<pin);			// Logic 1 for Open-Drain, Logic 0 for Relays
	}
}

// Function for Turning Loads ON and OFF by Relays Module

void On_Load(int x)
{
	if (x == 1)			// Reset State Load 1 ON
	{
		LCD_Clear_Then_Display("Relay Module ON");
		HAL_Delay(100);
		GPIOE->ODR &= ~(Output_1);
		GPIOE->ODR |=  (Output_2);
		GPIOE->ODR |=  (Output_3);
		GPIOE->ODR |=  (Output_4);
	}
	else if (x == 2)	// Load 2 ON
	{
		LCD_Clear_Then_Display("LOAD 2 is ON");
		HAL_Delay(100);
		GPIOE->ODR |=  (Output_1);
		GPIOE->ODR &= ~(Output_2);
		GPIOE->ODR |=  (Output_3);
		GPIOE->ODR |=  (Output_4);
	}
	else if (x == 3)	// Load 3 ON
	{
		LCD_Clear_Then_Display("LOAD 3 is ON");
		HAL_Delay(100);
		GPIOE->ODR |=  (Output_1);
		GPIOE->ODR |=  (Output_2);
		GPIOE->ODR &= ~(Output_3);
		GPIOE->ODR |=  (Output_4);
	}
	else if (x == 4)	// Load 4 ON
	{
		LCD_Clear_Then_Display("LOAD 4 is ON");
		HAL_Delay(100);
		GPIOE->ODR |=  (Output_1);
		GPIOE->ODR |=  (Output_2);
		GPIOE->ODR |=  (Output_3);
		GPIOE->ODR &= ~(Output_4);
	}
}

// Function for Testing Working of Relays in Module

void Relay_Test()
{
	if (GPIOA->IDR&0x1)
	{
		GPIOE->ODR |=  (Output_4);			// Load 4 OFF
		GPIOE->ODR &= ~(Output_1);			// Load 1 ON
		HAl_Delay(1000);
		GPIOE->ODR |=  (Output_1);			// Load 1 OFF
		GPIOE->ODR &= ~(Output_2);			// Load 2 ON
		HAl_Delay(1000);
		GPIOE->ODR |=  (Output_2);			// Load 2 OFF
		GPIOE->ODR &= ~(Output_3);			// Load 3 ON
		HAl_Delay(1000);
		GPIOE->ODR |=  (Output_3);			// Load 3 OFF
		GPIOE->ODR &= ~(Output_4);			// Load 4 ON
		HAL_Delay(1000);
	}
}
