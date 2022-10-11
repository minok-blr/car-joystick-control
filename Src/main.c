/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "math.h"
#include "MadgwickAHRS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define M_PI 3.14159265358979323846264338327950288
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#pragma GCC diagnostic ignored "-Wunused-value"
volatile int16_t meritev[10];	// array for saving data - package
volatile int8_t BINARY = 0;
volatile int8_t ASCII = 0;
volatile uint16_t packetCounter = 0x0000;
volatile uint16_t packetCounterASCII = 0;
//volatile float X_rad, Y_rad, Z_rad, X_acc, Y_acc, Z_acc, X_mag, Y_mag, Z_mag, roll, pitch, yaw;

volatile int8_t confidence = 0;
volatile int8_t buttonState = 0;
volatile int16_t sampleNum = 0;
volatile int16_t prevSample = 0;
volatile int16_t totalTime = 0;
volatile int8_t switchButton = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

uint8_t spi1_beriRegister(uint8_t);
void spi1_beriRegistre(uint8_t, uint8_t*, uint8_t);
void spi1_pisiRegister(uint8_t, uint8_t);
void initGyro(void);

uint8_t i2c1_pisiRegister(uint8_t, uint8_t, uint8_t);
void i2c1_beriRegistre(uint8_t, uint8_t, uint8_t*, uint8_t);
void initOrientation(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // "callback" funkcija za "overflow" periode na casovniku
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	// preveri ali je prekinitev klicana iz casovnika 3
	if (htim->Instance == TIM4) {
		uint8_t sample = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

		if(sample == 1)
		{
			if(sample != prevSample) confidence = 0;
			if(buttonState == 0)
			{
				// button was off
				// start counting to check the samples
				// sample is 1
				// increment confidence until 14(0 to 14)
				if(confidence < 15)
				{
					confidence++;
					if(confidence == 15)
					{
						// change button state and reset confidence
						buttonState = 1;
						confidence = 0;
						switchButton = 0;
					}
					totalTime++;
				}
			}

			else
			{
				totalTime++;
			}
			prevSample = sample;
		}

		if(sample == 0)
		{
			if(sample != prevSample) confidence = 0;
			if(buttonState == 0)
			{
				//do nothing
			}
			else // if button was on
			{
				if(confidence < 15)
				{
					confidence++;
					if(confidence == 15)
					{
						buttonState = 0;
						confidence = 0;
						switchButton = 1;
					}
				}
			}
			prevSample = sample;
		}
	}
	if (htim->Instance == TIM3) {

		if(BINARY == 1)
		{
			// accelerometer
			//i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&meritev[1], 6);
			// gyroscope
			//spi1_beriRegistre(0x28, (uint8_t*)&meritev[4], 6); // reg read raw x, y, z
			// magnetometer
			//i2c1_beriRegistre(0x1E, 0x68,(uint8_t*)&meritev[7], 6);

			//CDC_Transmit_FS((uint8_t*)&meritev, 20);
			//packetCounter = packetCounter + 1;
			//*(uint16_t*)&meritev[1] = (uint16_t)packetCounter;

			char msg[256] = {'\0'};
			packetCounterASCII++;
			sprintf(msg, "AI_CTRL: %d\n\r", packetCounterASCII);

			int var = 0;
			for (var; var < 256; ++var) {
			  if(msg[var] == '\0') break;
			}
			CDC_Transmit_FS((uint8_t*)&msg, var);
		}

		if(ASCII == 1)
		{

			// ACCELEROMETER
			i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&meritev[1], 6);

			// accelerometer in g
			float X_acc = (float)(meritev[1]>>4)*0.98f/1000.0f;
			float Y_acc = (float)(meritev[2]>>4)*0.98f/1000.0f;
			float Z_acc = (float)(meritev[3]>>4)*0.98f/1000.0f;

			// GYRO
			spi1_beriRegistre(0x28, (uint8_t*)&meritev[4], 6); // reg read raw x, y, z

//			X: -0.114444412291 0.236518442631 0.0696890170909 0.0686666443944
//			Y: -0.0534073933959 0.289925843477 0.0829493095838 0.0839259028435
//			Z: -0.205999940634 0.160222172737 -0.0266426590271 -0.0305185094476

			// gyro raw * 17.50 / 1000 to get dps minus the bias
			float X = (float)meritev[4] * 0.0175f - 0.0696890170909;
			float Y = (float)meritev[5] * 0.0175f - 0.0829493095838;
			float Z = (float)meritev[6] * 0.0175f + 0.0266426590271;

			// convert gyro dps to rads
			float X_rad = X*M_PI/180;
			float Y_rad = Y*M_PI/180;
			float Z_rad = Z*M_PI/180; // rad/s

			// MAGNETOMETER
			i2c1_beriRegistre(0x1E, 0x68,(uint8_t*)&meritev[7], 6);

//			Offsets:
//			X offset: -0.00514643038292
//			Y offset: 0.00309356147586
//			Z offset: -0.000687897983098
//			Scale factors:
//			X scale: 0.0120893912822
//			Y scale: 0.011347797381
//			Z scale: 0.011650735666

			// mag reading, calibrated and normalised
			float X_mag = (float)(meritev[7] + 0.00514643038292)  / 0.012089391282;
			float Y_mag = (float)(meritev[8] - 0.00309356147586)  / 0.011347797381;
			float Z_mag = (float)(meritev[9] + 0.000687897983098) / 0.011650735666;

			float  roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

			packetCounterASCII++;
			MadgwickAHRSupdate(-X_rad, Y_rad, -Z_rad, X_acc, Y_acc, Z_acc, X_mag, Y_mag, Z_mag, &roll, &pitch, &yaw);

			char msg[256] = {'\0'};
			sprintf(msg, "{\"IMU\":%d, \"roll\":%.3f, \"pitch\":%.3f, \"yaw\":%.3f}\n\r", packetCounterASCII, roll, pitch, yaw);

			int var = 0;
			for (var; var < 256; ++var) {
				if(msg[var] == '\0') break;
			}
			CDC_Transmit_FS((uint8_t*)&msg, var);
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		}
	}
}

// GYRO CODE

// Pavza, s katero omogocimo pravilno delovanje avtomatskega testa
void pavza(){
  uint32_t counter = 0;
  for(counter=0; counter<100; counter++){
    asm("nop");
  }
}

uint8_t spi1_beriRegister(uint8_t reg) {
  uint16_t buf_out, buf_in;
  reg |= 0x80; // najpomembnejsi bit na 1
  buf_out = reg; // little endian, se postavi na pravo mesto ....
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  pavza();
  //HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&buf_out, (uint8_t*)&buf_in, 2, 2); // blocking posiljanje ....
  HAL_SPI_TransmitReceive(&hspi1, &((uint8_t*)&buf_out)[0], &((uint8_t*)&buf_in)[0], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  pavza();
  HAL_SPI_TransmitReceive(&hspi1, &((uint8_t*)&buf_out)[1], &((uint8_t*)&buf_in)[1], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  pavza();
  return buf_in >> 8; // little endian...
}

void spi1_pisiRegister(uint8_t reg, uint8_t vrednost) {
  uint16_t buf_out;
  buf_out = reg | (vrednost<<8); // little endian, se postavi na pravo mesto ....
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  pavza();
  //HAL_SPI_Transmit(&hspi1, (uint8_t*)&buf_out, 2, 2); // blocking posiljanje ....
  HAL_SPI_Transmit(&hspi1, &((uint8_t*)&buf_out)[0], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  pavza();
  HAL_SPI_Transmit(&hspi1, &((uint8_t*)&buf_out)[1], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  pavza();
}

void spi1_beriRegistre(uint8_t reg, uint8_t* buffer, uint8_t velikost) {
  reg |= 0xC0; // najpomembnejsa bita na 1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  pavza();
  HAL_SPI_Transmit(&hspi1, &reg, 1, 10); // blocking posiljanje....
  pavza();
  HAL_SPI_Receive(&hspi1,  buffer, velikost, velikost); // blocking posiljanje....
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  pavza();
}

void initGyro() {
  // preverimo ali smo "poklicali" pravi senzor
  uint8_t cip = spi1_beriRegister(0x0F);
  if (cip!=0xD4 && cip!=0xD3) {
    for (;;);
  }
  spi1_pisiRegister(0x20, 0x0F); 	 // set PD, X, Y, Z registers to one, DR to 00, BW to 00
  spi1_pisiRegister(0x22, 0x08);	 // enable interrupt DRDY/INT2
  spi1_pisiRegister(0x23, 0x10);	 // set FS to 500dps, reg 0x23, hex value 0x10 = 0001 0000
}

// GYRO CODE END


// ACC/MAG CODE

uint8_t i2c1_pisiRegister(uint8_t naprava, uint8_t reg, uint8_t podatek) {
  naprava <<= 1;
  return HAL_I2C_Mem_Write(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, &podatek, 1, 10);
}

void i2c1_beriRegistre(uint8_t naprava, uint8_t reg, uint8_t* podatek, uint8_t dolzina) {
  if ((dolzina>1)&&(naprava==0x19))  // ce je naprava 0x19 moramo postaviti ta bit, ce zelimo brati vec zlogov
    reg |= 0x80;
  naprava <<= 1;
  HAL_I2C_Mem_Read(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, podatek, dolzina, dolzina);
}

void initOrientation() {
  HAL_Delay(10);

// Za potrebe testa, moramo testni napravi sporoviti kateri senzor imamo
//#define OLD_SENSOR 0x73 // Odkomentiramo za LSM303DLHC / stari senzorn
#define NEW_SENSOR 0x6E // Odkomentiramo za LSM303AGR / novi senzor

#if defined(OLD_SENSOR) && !defined(NEW_SENSOR)
  i2c1_pisiRegister(0x1e, 0x4F, OLD_SENSOR); // Povemo testni napravi, da imamo stari senzor
#elif !defined(OLD_SENSOR) && defined(NEW_SENSOR)
  i2c1_pisiRegister(0x1e, 0x4F, NEW_SENSOR); // Povemo testni napravi, da imamo novi senzor
#else
  for(;;); // V primeru napake, pocakamo tukaj
#endif
  HAL_Delay(100);

  // inicializiraj pospeskometer

  //i2c1_pisiRegister(0x19, 0x20, 0x27);  // zbudi pospeskometer in omogoci osi
  //i2c1_pisiRegister(0x19, 0x23, 0x88);  // nastavi posodobitev samo ko se prebere vrednost ter visoko locljivost
  i2c1_pisiRegister(0x19, 0x20, 0x57);  // response rate 100Hz
  i2c1_pisiRegister(0x19, 0x23, 0x88);  // sensitivity +-2g
  i2c1_pisiRegister(0x19, 0x22, 0x16);	// DRDY interrupt 0x8 or 0x16

  // init magnetometer
  i2c1_pisiRegister(0x1E, 0x60, 0x0C);  // REG_A_M set ODR to 11 - 100Hz
  i2c1_pisiRegister(0x1E, 0x62, 0x51);  // REG_C_M set INT_MAG to 1 and INT_MAG_PIN to 1?
  i2c1_pisiRegister(0x1E, 0x63, 0x01);  //
}

// ACC/MAG CODE END

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(&hspi1);
  __HAL_I2C_ENABLE(&hi2c1);
  HAL_TIM_Base_Start(&htim2); // for press counting, deprecated
  HAL_TIM_Base_Start_IT(&htim4); // for debouncing
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // CS postavimo na 1

  initGyro();
  initOrientation();

  meritev[0] = 0xaaab;	// package header

  uint32_t transmissionControl = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(buttonState == 1)
	  {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);

	  }
	  else // button state = 0
	  {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
    	if(switchButton == 1)
    	{

    		if(totalTime >= 27)
    		{
    			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
        		transmissionControl++;
        		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
    		}
    		//if(totalTime >= 370 && totalTime <= 970) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
    	}
		totalTime = 0;
	  }
	  if(transmissionControl == 4) transmissionControl = 0;

	  // IMU

	  if(transmissionControl == 1)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  BINARY = 1;
		  ASCII = 0;
		  HAL_TIM_Base_Start_IT(&htim3); // uncomment this line to start binary transmission
		  // new code for integrating the AI and hardware control
		  // comment out the HAL TIM START to stop the board from sending the binary data
		  // send only the word "AI_CTRL\r"
//			char msg[256] = {'\0'};
//			packetCounterASCII++;
//			sprintf(msg, "AI_CTRL: %d\n\r", packetCounterASCII);
//
//			int var = 0;
//			for (var; var < 256; ++var) {
//			  if(msg[var] == '\0') break;
//			}
//			CDC_Transmit_FS((uint8_t*)&msg, var);

		  //transmissionControl++;
		  //HAL_Delay(10);

	  }

	  if(transmissionControl == 2)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  BINARY = 0;
		  ASCII = 0;
		  HAL_TIM_Base_Stop_IT(&htim3);
		  packetCounterASCII = 0;
		  //char msg[8] = {'\0'};
		  //sprintf(msg, "EMPTY\r\n");
		  //CDC_Transmit_FS((uint8_t*)&msg, 8);
	  }

	  if(transmissionControl == 3)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  BINARY = 0;
		  ASCII = 1;
		  HAL_TIM_Base_Start_IT(&htim3);
		  //packetCounterASCII++;
	  }

	  if(transmissionControl == 0)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  BINARY = 0;
		  ASCII = 0;
		  HAL_TIM_Base_Stop_IT(&htim3);
		  //*(uint16_t*)&meritev[1] = (uint16_t)0x0000;
		  packetCounter = 0x0000;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 59999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 13;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
