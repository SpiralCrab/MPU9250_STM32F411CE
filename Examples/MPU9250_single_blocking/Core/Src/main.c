/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Gscale = GFS_2000DPS, Ascale = AFS_16G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x00;    // Variables for sensitivity, sampling rate etc.
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)


// Note that PI is defined in cmsis-dsp library as a macro. If you are not using that library, define it yourself or un-comment the line below
//const float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.1  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = 0.90689968211710892529703f;
float zeta = 0.0151149947019518154216f;

//volatile uint32_t int_flag1 = 0;

volatile uint32_t euler_flag = 0;


imu_t imu1 = {
            .hi2c = &hi2c1,
            .q = {1.0f, 0.0f, 0.0f, 0.0f},
            .bx = 1.0f,
            .bz = 0.0f,
            .gyroBias = {0.96, -0.21, 0.12},
            .accelBias = {0.00299, -0.00916, 0.00952},
            .magBias = {71.04, 122.43, -36.90},
            .magScale = {1.01, 1.03, 0.96},
            .MPU9250ID = MPU1
};
imu_t imu2 = {
            .hi2c = &hi2c1,
            .q = {1.0f, 0.0f, 0.0f, 0.0f},
            .bx = 1.0f,
            .bz = 0.0f,
            .gyroBias = {0.96, -0.21, 0.12},
            .accelBias = {0.00299, -0.00916, 0.00952},
            .magBias = {71.04, 122.43, -36.90},
            .magScale = {1.01, 1.03, 0.96},
            .MPU9250ID = MPU2
};
float Roll, Pitch, Yaw;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void updateIMU(imu_t* imu);
void paramInit(imu_t* imu, float gyroError, float gyroDrift );
void I2C_BusRelease(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){
  ITM_SendChar(ch);
  return(ch);
}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable TRC
    DWT->CYCCNT = 0;                                // Reset the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable the cycle counter
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
  MX_I2C1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  I2C_BusRelease();
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
  DWT_Init();
  uint8_t who1 = getMPU9250ID(&imu1);
  HAL_Delay(1000);

  paramInit(&imu1, 0.1, 0.00000435);            // Use to initialize beta and zeta with observed values. Change it if you need to

  uint8_t err_cnt = 0;
  while (who1 != 0x71){                         // Check "Who am I" register and retry 10 times.
    who1 = getMPU9250ID(&imu1);
    err_cnt++;
    HAL_Delay(300);
    if (err_cnt >10) Error_Handler();
  }

  resetMPU9250(&imu1);
  HAL_Delay(1000);

  aRes = getAres(Ascale);                       // Set the sensitivities and get the resolutions
  gRes = getGres(Gscale);
  mRes = getMres(Mscale);


  calibrateMPU9250(&imu1);

  HAL_Delay(1000);

  initMPU9250(&imu1, Ascale, Gscale, sampleRate, 0);        // Setting the fifth argument 1 will enable the interrupt signals on MPU9250.

  who1 = getAK8963CID(&imu1);                               // Checking the magnetometer's "Who am I" register
  HAL_Delay(1000);

  initAK8963Slave(&imu1, Mscale, Mmode);                    // Set the magnetometer as slave, MPU9250 as master (to separate magnetometers when multiple MPU9250 is used)


  DWT_Init();
  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (euler_flag){
      euler_flag = 0;
      if (readByte(MPU1, INT_STATUS) & 0x01) updateIMU(&imu1);
      Roll = imu1.roll;
      Pitch = imu1.pitch;
      Yaw = imu1.yaw;
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 *  @fn void paramInit(imu_t* imu, float gyroError, float gyroDrift)
 *  @brief
 */
void paramInit(imu_t* imu, float gyroError, float gyroDrift ){
  imu->GyroMeasError = PI * (gyroError/180.0f);
  imu->GyroMeasDrift = PI * (gyroDrift/180.0f);
  imu->beta = sqrtf(3.0f/4.0f)*imu->GyroMeasError;
  imu->zeta = sqrtf(3.0f/4.0f)*imu->GyroMeasDrift;
}


/**
 *  @fn void updateIMU(imu_t* imu)
 *  @brief A function that gets accel, gyro, magneto data and use Madgwick's filter to get AHRS euler angles.
 *  @note You can either use MadgwickQuaternionUpdate() or Madgwick2(). The former is directly from Kris Winer's code, the latter is from Madgwick's internal report released in 2010 \n
 *  The former filter does not use zeta value thus does not calibrate the gyro drift for long-term use. The latter one uses zeta to compensate gyro drift.
 *
 */
void updateIMU(imu_t* imu){
 /*Get raw accel and gyro data*/
  readMPU9250Data(imu);

  imu->ax = (float)imu->MPU9250Data[0]*aRes - imu->accelBias[0];
  imu->ay = (float)imu->MPU9250Data[1]*aRes - imu->accelBias[1];
  imu->az = (float)imu->MPU9250Data[2]*aRes - imu->accelBias[2];

  imu->gx = (float)imu->MPU9250Data[4]*gRes;
  imu->gy = (float)imu->MPU9250Data[5]*gRes;
  imu->gz = (float)imu->MPU9250Data[6]*gRes;

 /*get magnetometer data*/
  readMagData(imu);

  imu->mx = (float)imu->magCount[0]*mRes* imu->magCalibration[0] - imu->magBias[0];
  imu->my = (float)imu->magCount[1]*mRes* imu->magCalibration[1] - imu->magBias[1];
  imu->mz = (float)imu->magCount[2]*mRes* imu->magCalibration[2] - imu->magBias[2];

  imu->mx *= imu->magScale[0];
  imu->my *= imu->magScale[1];
  imu->mz *= imu->magScale[2];

  /*get the delta time for the numerical integration*/
  imu->Now = DWT->CYCCNT;
  uint32_t dt_us = imu->Now - imu->LastUpdate;
  imu->LastUpdate = imu->Now;
  imu->deltat = (float)dt_us / (float)SystemCoreClock;

  /*Use Madgwick filter and update quaternions, and get roll pitch yaw in radian*/
//  MadgwickQuaternionUpdate(imu);
  Madgwick2(imu);
  float q1 = imu->q[0], q2 = imu->q[1], q3 = imu->q[2], q4 = imu->q[3];
//  arm_atan2_f32(2.0f * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4, &imu->yaw);
  imu->yaw = atan2f(2.0f * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
  imu->pitch = -asinf(2.0f * (q2 * q4 - q1 * q3));
//  arm_atan2_f32(2.0f * (q1 * q2 + q3 * q4), q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4, &imu->roll);
  imu->roll = atan2f(2.0f * (q1 * q2 + q3 * q4), q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);

  /*Convert roll pitch yaw from radian to degrees*/
  imu->pitch *= 180.0f / PI;
  imu->yaw *= 180.0f / PI;
  imu->yaw += 8.58f; // Declination at Seoul, Korea. 2025-04-28
  imu->roll *= 180.0f / PI;

}

/**
 * @fn void I2C_BusRelease(void)
 * @brief A function that frees the I2C bus when it is locked up
 * @note Use HAL_I2C_DeInit and HAL_I2C_Init right after using this function
 */
void I2C_BusRelease(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  for (int i = 0; i < 9; i++){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);
}


/**
 * @brief EXTI based callback when MPU9250 is set to interrupt mode. This feature is not used in this example
 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//  if (GPIO_Pin == GPIO_PIN_1) int_flag1 = 1;
//}

/**
 * @brief Timer interrupt based callback. This interrupt will occur every 5 ms.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM10){
    euler_flag = 1;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){

}


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
