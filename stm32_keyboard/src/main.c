

/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : main.c
	* @brief          : Main program body
	******************************************************************************
	* @attention
	*
	* Copyright (c) 2024 STMicroelectronics.
	* All rights reserved.
	*
	* This software is licensed under terms that can be found in the LICENSE file
	* in the root directory of this software component.
	* If no LICENSE file comes with this software, it is provided AS-IS.
	*
	******************************************************************************
	*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_hid.h"

#define NORMAL_KEY 1  
#define MEDIA_KEY 2 

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct 
{
	uint8_t ID;
 	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
} keyboardReportDes;

typedef struct 
{
	uint8_t ID;
 	uint8_t MEDIA;
} keyboardMediaDes;



char msg_buff[100] = {0};

uint16_t Ports[16] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};


/*
A\B |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  10  |  11  |  12  |  13  |  14  |  15  |  16  |  17  |
0   |  F7 |Pause|     |     |LCtrl|     |Menu |  M  |  \  |Space|  F6  | Left |      | LAlt |  Win |      |      |  Fn  |
1   |  F3 |  Up |     |  .  |RCtrl|LShif|     |  K  |  N  |  V  |  F2  | Right|      |      |      |  D   |      |  Z   |
2   |   1 |Prtsc|     |     |     |     |     |  ,  |  B  |  C  |Capslk| Down |      | RAlt |      |  X        
3   |  F4 |  2  |     |  /  |     |RShif|     |  L  |  J  |  G  |  F1  | PgDw |      |      |      |  F   |      |  A   |
4   |   Q |  3  |     |  ;  |     |     |Enter|  I  |  Y  |  H  | Tab  | PgUp |      |      |      |  E   |      |  S   |   
5   |  F12|  F9 |     |  P  |     |     |  =  |  0  |  8  |  6  |  ~   |  del |      |      |      |  4   |
6   |  F8 |Bacsp|     |  '  |     |     |  ]  |  O  |  U  |  T  | F5   |      |      |      |      |  R   |      |  W   |
7   |  F11|  F10|     |  [  |     |     |     |  -  |  9  |  7  | Esc  |  ins |      |      |      |  5   |
*/



typedef struct
{
	int Port;
	uint16_t PinNum;
	uint8_t KEYCODE1;
	uint8_t MODIFIER;
	uint8_t FUNK_KEYCODE;
	uint8_t FUNK_MEDIA;
	uint8_t FUNK_KEY;
} KeyDesk;

enum GPIO_PORT
{
	PORT_A,
	PORT_B
};

KeyDesk Fn = {PORT_A, GPIO_PIN_8, 0x00, 0b00000000, 0x00, 0b00000000, 0x01};
KeyDesk Key_F7 = {PORT_B, GPIO_PIN_0, 0x40, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Pause = {PORT_B, GPIO_PIN_1, 0x48, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk LCtrl = {PORT_B, GPIO_PIN_4, 0x00, 0b00000001, 0x00, 0b00000000, 0x00};
KeyDesk Menu = {PORT_B, GPIO_PIN_6, 0x65, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_M = {PORT_B, GPIO_PIN_7, 0x10, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_dash = {PORT_B, GPIO_PIN_8, 0x31, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Space = {PORT_B, GPIO_PIN_9, 0x2c, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_F6 = {PORT_B, GPIO_PIN_10, 0x3F, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Left = {PORT_B, GPIO_PIN_11, 0x50, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk LAlt = {PORT_B, GPIO_PIN_13, 0x00, 0b00000100, 0x00, 0b00000000, 0x00};
KeyDesk Win = {PORT_B, GPIO_PIN_14, 0x00, 0b00001000, 0x00, 0b00000000, 0x00};

KeyDesk Key_F3 = {PORT_B, GPIO_PIN_0, 0x3c, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Up = {PORT_B, GPIO_PIN_1, 0x52, 0b00000000, 0x00, 0b00100000, 0x00};
KeyDesk Key_Dot = {PORT_B, GPIO_PIN_3, 0x37, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk RCtrl = {PORT_B, GPIO_PIN_4, 0x00, 0b00010000, 0x00, 0b00000000, 0x00};
KeyDesk LShif = {PORT_B, GPIO_PIN_5, 0x00, 0b00000010, 0x00, 0b00000000, 0x00};
KeyDesk Key_K = {PORT_B, GPIO_PIN_7, 0x0e, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_N = {PORT_B, GPIO_PIN_8, 0x11, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_V = {PORT_B, GPIO_PIN_9, 0x19, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_F2 = {PORT_B, GPIO_PIN_10, 0x3b, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Right = {PORT_B, GPIO_PIN_11, 0x4f, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_D = {PORT_B, GPIO_PIN_15, 0x07, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Z = {PORT_A, GPIO_PIN_8, 0x1d, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk Key_1 = {PORT_B, GPIO_PIN_0, 0x1e, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Prtsc = {PORT_B, GPIO_PIN_1, 0x46, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Comma = {PORT_B, GPIO_PIN_7, 0x36, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_B = {PORT_B, GPIO_PIN_8, 0x05, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_C = {PORT_B, GPIO_PIN_9, 0x06, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Capslk = {PORT_B, GPIO_PIN_10, 0x39, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Down = {PORT_B, GPIO_PIN_11, 0x51, 0b00000000, 0x00, 0b01000000, 0x00};
KeyDesk RAlt = {PORT_B, GPIO_PIN_13, 0x00, 0b01000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_X = {PORT_B, GPIO_PIN_15, 0x1b, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk Key_F4 = {PORT_B, GPIO_PIN_0, 0x3d, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_2 = {PORT_B, GPIO_PIN_1, 0x1f, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Slash = {PORT_B, GPIO_PIN_3, 0x38, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk RShif = {PORT_B, GPIO_PIN_5, 0x00, 0b00100000, 0x00, 0b00000000, 0x00};
KeyDesk Key_L = {PORT_B, GPIO_PIN_7, 0x0f, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_J = {PORT_B, GPIO_PIN_8, 0x0d, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_G = {PORT_B, GPIO_PIN_9, 0x0a, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_F1 = {PORT_B, GPIO_PIN_10, 0x3a, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk PgDw = {PORT_B, GPIO_PIN_11, 0x4e, 0b00000000, 0x4d, 0b00000000, 0x00};
KeyDesk Key_F = {PORT_B, GPIO_PIN_15, 0x09, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_A = {PORT_A, GPIO_PIN_8, 0x04, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk Key_Q = {PORT_B, GPIO_PIN_0, 0x14, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_3 = {PORT_B, GPIO_PIN_1, 0x20, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_semicolon = {PORT_B, GPIO_PIN_3, 0x33, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Enter = {PORT_B, GPIO_PIN_6, 0x28, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_I = {PORT_B, GPIO_PIN_7, 0x0c, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Y = {PORT_B, GPIO_PIN_8, 0x1c, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_H = {PORT_B, GPIO_PIN_9, 0x0b, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Tab = {PORT_B, GPIO_PIN_10, 0x2b, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk PgUp = {PORT_B, GPIO_PIN_11, 0x4b, 0b00000000, 0x4a, 0b00000000, 0x00};
KeyDesk Key_E = {PORT_B, GPIO_PIN_15, 0x08, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_S = {PORT_A, GPIO_PIN_8, 0x16, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk Key_F12 = {PORT_B, GPIO_PIN_0, 0x45, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_F9 = {PORT_B, GPIO_PIN_1, 0x42, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_P = {PORT_B, GPIO_PIN_3, 0x13, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_equals = {PORT_B, GPIO_PIN_6, 0x2e, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_0 = {PORT_B, GPIO_PIN_7, 0x27, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_8 = {PORT_B, GPIO_PIN_8, 0x25, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_6 = {PORT_B, GPIO_PIN_9, 0x23, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_tilde = {PORT_B, GPIO_PIN_10, 0x35, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_del = {PORT_B, GPIO_PIN_11, 0x4c, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_4 = {PORT_B, GPIO_PIN_15, 0x21, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk Key_F8 = {PORT_B, GPIO_PIN_0, 0x41, 0b00000000, 0x00, 0b00010000, 0x00};
KeyDesk Bacsp = {PORT_B, GPIO_PIN_1, 0x2a, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_quotes = {PORT_B, GPIO_PIN_3, 0x34, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Rbracket = {PORT_B, GPIO_PIN_6, 0x30, 0b00000000, 0x00, 0b00000000, 0x00}; 
KeyDesk Key_O = {PORT_B, GPIO_PIN_7, 0x12, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_U = {PORT_B, GPIO_PIN_8, 0x18, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_T = {PORT_B, GPIO_PIN_9, 0x17, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_F5 = {PORT_B, GPIO_PIN_10, 0x3e, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_R = {PORT_B, GPIO_PIN_15, 0x15, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_W = {PORT_A, GPIO_PIN_8, 0x1a, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk Key_F11 = {PORT_B, GPIO_PIN_0, 0x44, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_F10 = {PORT_B, GPIO_PIN_1, 0x43, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Lbracket = {PORT_B, GPIO_PIN_3, 0x2f, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Minus = {PORT_B, GPIO_PIN_7, 0x2d, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_9 = {PORT_B, GPIO_PIN_8, 0x26, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_7 = {PORT_B, GPIO_PIN_9, 0x24, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Esc = {PORT_B, GPIO_PIN_10, 0x29, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_Ins = {PORT_B, GPIO_PIN_11, 0x49, 0b00000000, 0x00, 0b00000000, 0x00};
KeyDesk Key_5 = {PORT_B, GPIO_PIN_15, 0x22, 0b00000000, 0x00, 0b00000000, 0x00};

KeyDesk* KeyRows[8][12] = 
{
	{&Fn, &Key_F7, &Pause, &LCtrl, &Menu, &Key_M, &Key_dash, &Space, &Key_F6, &Left, &LAlt, &Win},
	{&Key_F3, &Up, &Key_Dot, &RCtrl, &LShif, &Key_K, &Key_N, &Key_V, &Key_F2, &Right, &Key_D, &Key_Z},
	{&Key_1, &Prtsc, &Comma, &Key_B, &Key_C, &Capslk, &Down, &RAlt, &Key_X, NULL, NULL, NULL},
	{&Key_F4, &Key_2, &Key_Slash, &RShif, &Key_L, &Key_J, &Key_G, &Key_F1, &PgDw, &Key_F, &Key_A, NULL},
	{&Key_Q, &Key_3, &Key_semicolon, &Enter, &Key_I, &Key_Y, &Key_H, &Key_Tab, &PgUp, &Key_E, &Key_S, NULL},
	{&Key_F12, &Key_F9, &Key_P, &Key_equals, &Key_0, &Key_8, &Key_6, &Key_tilde, &Key_del, &Key_4, NULL, NULL},
	{&Key_F8, &Bacsp, &Key_quotes, &Rbracket, &Key_O, &Key_U, &Key_T, &Key_F5, &Key_R, &Key_W, NULL, NULL},
	{&Key_F11, &Key_F10, &Lbracket, &Key_Minus, &Key_9, &Key_7, &Key_Esc, &Key_Ins, &Key_5, NULL, NULL, NULL}
};

uint8_t IsKeyPressed = 0;
uint8_t IsMediaKeyPressed = 0;
uint32_t  TimmeFromDataSend = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void KeyBoardPrint(char *data,uint16_t length);
void AddKey(keyboardReportDes *Msg, u_int8_t KEYCODE, int num);




/**
	* @brief  The application entry point.
	* @retval int
	*/
int main(void)
{
 

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();

	/* Infinite loop */
	
	while (1)
	{
		uint8_t Keys[5] = {0,0,0,0,0};
		uint8_t Modifer = 0;
		keyboardMediaDes HIDMediaMessage = {MEDIA_KEY,0};

		uint8_t KeyNum = 0;
		uint8_t KeyMod = 0;
		uint8_t MediaKeyPress = 0;
		uint8_t FnPress = 0;
		GPIO_PinState CurentState;

		for (int i = 0; i < 8; i++)
		{
			
			HAL_GPIO_WritePin(GPIOA, Ports[i], GPIO_PIN_RESET);
			
			HAL_Delay(7);
			
			for (int j = 0; j < 12; j++)
			{
				
				
				if (KeyRows[i][j])
				{
					
					if (KeyRows[i][j]->Port == PORT_A)
					{
						CurentState = HAL_GPIO_ReadPin(GPIOA, KeyRows[i][j]->PinNum);
					}
					else 
					{
						CurentState = HAL_GPIO_ReadPin(GPIOB, KeyRows[i][j]->PinNum);
					}
					
					if (!CurentState)
					{
						if (!FnPress)
						{
							if (KeyRows[i][j]->KEYCODE1)
							{
								Keys[KeyNum] = KeyRows[i][j]->KEYCODE1;
								if (KeyNum < 4) KeyNum++;

							}
							else if (KeyRows[i][j]->MODIFIER)
							{
								Modifer |= KeyRows[i][j]->MODIFIER;
								KeyMod = 1;
							}
							else if (KeyRows[i][j]->FUNK_KEY) 
							{
								FnPress = 1;
							}
						}
						else
						{
							if (KeyRows[i][j]->FUNK_KEYCODE)
							{
								Keys[KeyNum] = KeyRows[i][j]->FUNK_KEYCODE;
								if (KeyNum < 4) KeyNum++;
							}
							else if (KeyRows[i][j]->FUNK_MEDIA)
							{
								HIDMediaMessage.MEDIA |= KeyRows[i][j]->FUNK_MEDIA;
								MediaKeyPress = 1;
							}
						}
						
					} 
				}

			}
			
			HAL_GPIO_WritePin(GPIOA, Ports[i], GPIO_PIN_SET);
		} 
		
		
		
		if(KeyNum || KeyMod)
		{
			keyboardReportDes Message = {NORMAL_KEY,Modifer,0,Keys[0],Keys[1],Keys[2],Keys[3],Keys[4]};
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&Message, sizeof(Message));
			IsKeyPressed = 1;
				
		}
		else if (MediaKeyPress)
		{
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&HIDMediaMessage, sizeof(HIDMediaMessage));
			IsMediaKeyPressed = 1;
		}
		else if (IsKeyPressed)
		{
			keyboardReportDes BlankKey = {NORMAL_KEY,0,0,0,0,0,0,0};
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&BlankKey, sizeof(BlankKey));
			IsKeyPressed = 0;
		}
		else if (IsMediaKeyPressed)
		{
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&HIDMediaMessage, sizeof(HIDMediaMessage));
			IsMediaKeyPressed = 0;
		}

			
		
	
	}
}


/**
	* @brief System Clock Configuration
	* @retval None
	*/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
	Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
	Error_Handler();
	}
}

/**
	* @brief GPIO Initialization Function
	* @param None
	* @retval None
	*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
							|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

	/*Configure GPIO pins : PA0 PA1 PA2 PA3
							 PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
							|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
							 PB11 PB12 PB13 PB14
							 PB15 PB3 PB4 PB5
							 PB6 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
							|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
							|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
							|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}



/**
	* @brief  This function is executed in case of error occurrence.
	* @retval None
	*/
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
	* @brief  Reports the name of the source file and the source line number
	*         where the assert_param error has occurred.
	* @param  file: pointer to the source file name
	* @param  line: assert_param error line source number
	* @retval None
	*/
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


void KeyBoardPrint(char *data,uint16_t length)
{
	//How to use
	//sprintf(msg_buff, "Hellow sdfsdffsfefrwyfgjtyjtkullfsergesdfaswqewdffsfsefwrwer \n");
	//KeyBoardPrint(msg_buff, strlen(msg_buff));


	keyboardReportDes keyBoardHIDsub = {NORMAL_KEY,0,0,0,0,0,0,0};

	for(uint16_t count=0;count<length;count++)
	{
		if(data[count]>=0x41 && data[count]<=0x5A) //Capital leter
		{
			keyBoardHIDsub.MODIFIER=0x02;
			keyBoardHIDsub.KEYCODE1=data[count]-0x3D;
		}
		else if(data[count]>=0x61 && data[count]<=0x7A) keyBoardHIDsub.KEYCODE1=data[count]-0x5D; // lowercase leter
		
		else if(data[count]==0x20) keyBoardHIDsub.KEYCODE1=0x2C; //Space
		
		else if(data[count]==0x0A) keyBoardHIDsub.KEYCODE1=0x28; //Enter
	
		else if (data[count] == 0x30) keyBoardHIDsub.KEYCODE1 = 0x27; //Zero
	 
		else if (data[count] >= 0x31 && data[count] <= 0x39) keyBoardHIDsub.KEYCODE1 = 0x1d + data[count] - 0x30; //Digit
	 
		else if (data[count] == 0x3d) keyBoardHIDsub.KEYCODE1 = 0x2e; //=

		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t *) &keyBoardHIDsub,sizeof(keyBoardHIDsub));
		HAL_Delay(15);
		keyBoardHIDsub.MODIFIER=0x00;
		keyBoardHIDsub.KEYCODE1=0x00;
		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t *) &keyBoardHIDsub,sizeof(keyBoardHIDsub));

		HAL_Delay(25);
	}
}



