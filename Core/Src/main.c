/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_tim.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_DUTY 9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t  time_old=0;
float target_ang=1.0;
float gyro_ang = 0; //angle derived from integration
float  comp_ang =0; //complimentary angle derived from both gyro and acceleration sensor

float Kp = 30;          // very stiff: full authority by ~3 deg
float Kd = 0.4;         // heavy damping to survive the stiffness
float Ki = 0;           // out of the picture entirely

float gyro_f = 0;

float integral =0;

//define integer 32bits
uint32_t loop_count=0;

    //CAlIBRATION OF ACCELEROMETER OFFSETS X Y AND Z:


    //DEFINE CONSTANTS AND DECLAER

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//gyro around y axis is pitch in our case, around x roll and z yaw, x and z gyro should be zero
void readIMU(int16_t *aX, int16_t *aY, int16_t *aZ, int16_t *gX, int16_t *gY, int16_t *gZ){

    uint8_t accel_data[6];

    //read accel data
    HAL_I2C_Mem_Read(&hi2c1, 0x68<<1,0x3B, 1, accel_data, 6, 100);


    uint8_t gyro_data[6];

    //read gyro data
    HAL_I2C_Mem_Read(&hi2c1, 0x68<<1,0x43, 1, gyro_data, 6, 100);

*aX=(accel_data[0]<<8) |accel_data[1];

*aY=(accel_data[2] <<8) |accel_data[3];

*aZ=(accel_data[4] <<8) |accel_data[5];

*gX=(gyro_data[0] <<8) |gyro_data[1];

*gY=(gyro_data[2] <<8) |gyro_data[3];

*gZ=(gyro_data[4] <<8) |gyro_data[5];
}

float PID(float dt,float comp_ang,float gyro_Z_deg){

    //PID LOOP
     //1 is the balance point for the robot as center of mass is slightly in the back
    float error = target_ang - comp_ang;

    //Kp constant = stiffeness of the spring with respect to current error size
    float P=Kp*error;

    //Kd constant = stiffeness with respect to speed of error changing
    float D = Kd*(-gyro_Z_deg);

    //Ki stiffeness/ reaction to KI error

    


 if (error > 1.5f || error < -1.5f){
       integral += error*dt;
   if (integral > 60) integral = 60;
   if (integral < -60) integral = -60;
 }

    float I = Ki*integral;
    
    
    //adding response
    return(P+I+D);
}
    
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // wake up the MPU6050

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //starts pin that connect to motor controller
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //starts pin that connect to motor controller
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  uint8_t wakeup = 0; //creates variable with assiged value 0

  uint8_t dlpf_setting= 3; // 44Hz

  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1,0x1A,1,&dlpf_setting,1,100);

  // peripherrral name, mpu6050s address, register (power maagemet rergister), the adrress size (1 byte), poiter to the data im sending (wakekup varrriable is 0), how many bittes o send (1), timeoutt i ms (100ms++ imeou)
  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1,0x6B,1,&wakeup,1,100); 


    //CALIBRATE GYRO ON STARTUP WHILE NOTHING IS TILTING
      //LOOP THAT TAKES 500 READINGS OF GYRO AND REMOVIES BIAS (IF FLAT THEN X AND Y)


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    
  {
    uint32_t now=HAL_GetTick();

    float dt=(now-time_old)/ 1000.0; //.0 is important to keep it a float

    time_old=now;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    
  //read accelerometer code

    //create buffer

      int16_t accel_X;

      int16_t accel_Y;

      int16_t accel_Z;

      int16_t gyro_X;

      int16_t gyro_Y;

      int16_t gyro_Z;

      readIMU(&accel_X, &accel_Y, &accel_Z, &gyro_X, &gyro_Y, &gyro_Z);

      float angle_accel;
    angle_accel= atan2(-accel_Y,-accel_X);

      float angle_accel2;
    angle_accel2= atan2(accel_Y,-accel_X);

      float angle_deg_accel;

    angle_deg_accel=(angle_accel*180)/3.14159;

      float angle_deg_accel2;
    angle_deg_accel2=(angle_accel2*180)/3.14159;

      float gyro_Z_deg; //gyro in deg/s

gyro_Z_deg = -gyro_Z/131.0f;                 // (your existing line, sign included)
gyro_f = 0.7f * gyro_f + 0.3f * gyro_Z_deg;  // smoothing AFTER the value exists //output for gyro gets sent as 131/degree of rotation

      //integrate to get angle purely based on gyro
      
       gyro_ang = gyro_ang + gyro_f*dt; 

      //integrate based on complimentary filter
      
        comp_ang = (comp_ang + (gyro_f*dt))*0.98 + angle_deg_accel*0.02;

      
      

    //print over UART
      //creates a buffer forr the text
    char msg[128]; 
      //replace each space by a number in order of X Y Z X...
    loop_count++;




      

    float output= -PID(dt,comp_ang,gyro_f);
    float mag = output; //mag is how much powerr is set ot motorr (magitude of pulse width)

    //absolute value
    if (mag<0){
    mag = -mag;
    }  
    //clamp magnitude 
    if (mag > 99){
      mag=90; //max duty is 99 but we'll clamp at 99
    }
    
    int duty=0;

    float err_now = comp_ang - target_ang;
        if (err_now > -2.5f && err_now < 2.5f && gyro_f > -40 && gyro_f < 40){
    duty = 0;
} else {
        duty = (int)(MIN_DUTY + mag * (99.0f - MIN_DUTY) / 90.0f);
    }




    if (mag<5){
     duty =0;
    }
    else{
    duty = (int)(MIN_DUTY + (mag - 5) * (99.0 - MIN_DUTY) / 85.0);
    }
    //safety switch (more than 45 means robot cant be saved so just stop spinning)

    if (comp_ang>45 || comp_ang<-45){
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 99);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 99);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 99);
     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 99);
    }
        
    //if output is positive, spin forwards
    else{
      if (output>=0){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 99-duty); //timer 2 chanel 1, PWM and two
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 99);
      //spin backwards
      }
      else if (output<0){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 99-duty); //timer 2 chanel 1, PWM and two
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 99);
      }

      //motor B
      if (output>=0){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 99-duty); //timer 2 chanel 1, PWM and two
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 99);
      }
      else if (output<0){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 99-duty); //timer 2 chanel 1, PWM and two
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 99);
      }
    }
    if (loop_count>=50){ //every 50 loops (250 ms with 5ms delay)
      loop_count=0; //reset loop count
    
    //builds the strink to send over uart

sprintf(msg,"V12 a:%d c:%d g:%d out:%d d:%d\r\n",
    (int)angle_deg_accel,(int)comp_ang,(int)gyro_f,(int)output,(int)duty);

    //sprintf(msg,"Angle(acc): %d Angle(gyr): %d Angle_comp: %d inst_angle_gyro: %d (deg/s)\r\n ",(int)angle_deg_accel,(int)gyro_ang, (int)comp_ang, (int)gyro_Z_deg);

      //Send text over UART:
      //configured uart, text to send in bites, length of message (string), timeout if no response
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    } //only prints 4 times/s

    HAL_Delay(5); //only 5ms delay to make frequency 200HZ


    //read values when lying in each of 6 axis to get offset in acceleration


    //values/16384 * 9.81 = accel in m/s^2


    //make complementary filter out of gyro and accel angle
    //0.98*(gyro current angle + dt*gyro(instantaneous))+0.02*(tan2(xaccel, zaccel)*180/3.14159)   we converted from rad to degrees

    //atan2 is needed to distinguish between - angles and + angles




    //PID loop and 3 constants: actually control the wheels and send commands
  /* USER CODE END 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
