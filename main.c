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
#include "arm_math.h"
#include "time.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CORDIC_HandleTypeDef hcordic;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t SpeedSet = 100;
int16_t TimeSet = 10000;
int16_t n = 0;

float SPEED, speed, P, I, SPEED_expected;
qd_t a;
float cof=0.3;
float B_IIR[3]={1,2,1}; //分子
float A_IIR[3]={1, 1.14298050254, 0.4128015980962}; //分母

int history_speed_IIR_origin[2]={0,0};
int history_speed_IIR_filterd[2]={0,0};
float tmp1;
float const_1=1.0;
float sum=0;
float FIR_cof[6]={0.01861755945526,  -0.1146286727783,    0.596290909881,    0.596290909881, -0.1146286727783,  0.01861755945526};
float tmp2;
float da_e;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
struct angle_thing {
    int16_t angle_en, angle_el;
} Angle;

float Get_Speed() {

    Angle.angle_en = MC_GetElAngledppMotor1();
    if (Angle.angle_en < Angle.angle_el) {
        da_e = (float) (65535 - Angle.angle_el + Angle.angle_en);
    } else {
        da_e = (float) (Angle.angle_en - Angle.angle_el);
    }
    Angle.angle_el = Angle.angle_en;

    return da_e * 0.026157;
}




int filter_FIR5(int current_speed,int *history_speed){
	int i=0;
	for(i=0;i<5;i++){
		if (history_speed[i]==0){
			history_speed[i]=current_speed;
			return current_speed;
		}
	}
	current_speed=current_speed*FIR_cof[0]+history_speed[0]*FIR_cof[1]+history_speed[1]*FIR_cof[2]+history_speed[2]*FIR_cof[3]+history_speed[3]*FIR_cof[4]+history_speed[4]*FIR_cof[5]+history_speed[5]*FIR_cof[6];
	for(i=0;i<4;i++){
		history_speed[i]=history_speed[i+1];
	}
	history_speed[i]=current_speed;
	return current_speed;
}



int filter_IIR2(int current_speed){
	if (history_speed_IIR_origin[1]==0){
		history_speed_IIR_origin[1]=current_speed;
		history_speed_IIR_filterd[1]=current_speed;
		return current_speed;
	}else if(history_speed_IIR_origin[0]==0){
		history_speed_IIR_origin[0]=current_speed;
		history_speed_IIR_filterd[0]=current_speed;
		return current_speed;
	}
	int filtered_speed=(int)(B_IIR[0]*current_speed+B_IIR[1]*history_speed_IIR_origin[0]+B_IIR[2]*history_speed_IIR_origin[1]-A_IIR[1]*history_speed_IIR_filterd[0]+A_IIR[2]*history_speed_IIR_filterd[1]);
	history_speed_IIR_origin[1]=history_speed_IIR_origin[0];
	history_speed_IIR_filterd[1]=history_speed_IIR_filterd[0];
	history_speed_IIR_origin[0]=current_speed;
	history_speed_IIR_filterd[0]=filtered_speed;
	return filtered_speed;
}

float filter_average(float current_speed,float *history_speed,int size){
//	int sum=0;
	sum=0;
	int i=0;
	for(i=0;i<size;i++){
		if (history_speed[i]==0){
			break;
		}else{
			sum+=history_speed[i];
		}
	}
	current_speed=(sum+current_speed)/(i+1);
//	放入current_speed
	if (i==size){
		for(i=0;i<size-1;i++){
			history_speed[i]=history_speed[i+1];
		}
		history_speed[size-1]=current_speed;
	}else{
		history_speed[i]=current_speed;
	}
	return current_speed;
}

float filter_average_dsp(float current_speed,float *history_speed,int size){
	sum=0;
	int i=0;
	for(i=0;i<size;i++){
		if (history_speed[i]==0){
			break;
		}else{
			arm_add_f32(&sum,&(history_speed[i]),&sum,1);
//			sum+=history_speed[i];
		}
	}
	arm_add_f32(&sum,&current_speed,&tmp1,1);
	current_speed=tmp1/(i+1);
//	放入current_speed
	if (i==size){
		for(i=0;i<size-1;i++){
			history_speed[i]=history_speed[i+1];
		}
		history_speed[size-1]=current_speed;
	}else{
		history_speed[i]=current_speed;
	}
	return current_speed;
}


float filter_RC(float current_speed,float last_speed){
	if (last_speed!=0)
		current_speed = cof * (float)current_speed + (1.000f - cof) * last_speed;
	return current_speed;
}

float filter_RC_dsp(float current_speed,float last_speed){
	if (last_speed!=0){
		arm_mult_f32(&current_speed,&cof,&current_speed,1);
		arm_sub_f32(&const_1,&cof,&tmp1,1);
		arm_mult_f32(&last_speed,&tmp1,&tmp1,1);
		arm_add_f32(&tmp1,&current_speed,&current_speed,1);
//		current_speed = current_speed + (1.000f - cof) * last_speed;
	}
	return current_speed;
}

float filter_realtime(int filter_mode, float current_speed, float *history_speed, int size, float last_speed){
	if (filter_mode==1){
		return filter_average(current_speed, history_speed, size);
	}else if(filter_mode==2){
		return filter_RC(current_speed, last_speed);
	}else if(filter_mode==3){
		return filter_IIR2(current_speed);
	}else if(filter_mode==4){
		return filter_FIR5(current_speed,history_speed);
	}
}

float filter_realtime_dsp(int filter_mode, float current_speed, float *history_speed, int size, float last_speed){
	if (filter_mode==1){
		return filter_average(current_speed, history_speed, size);
	}else if(filter_mode==2){
		return filter_RC_dsp(current_speed, last_speed);
	}else if(filter_mode==3){
		return filter_IIR2(current_speed);
	}else if(filter_mode==4){
		return filter_FIR5(current_speed,history_speed);
	}
}


int UI_HandleStartStopButton_cb (int flag)
{
/* USER CODE BEGIN START_STOP_BTN */
//  if ((IDLE == MC_GetSTMStateMotor1())&&(HAL_GPIO_ReadPin(Start_Stop_GPIO_Port,Start_Stop_Pin) == GPIO_PIN_SET))
	if(HAL_GPIO_ReadPin(Start_Stop_GPIO_Port,Start_Stop_Pin) == GPIO_PIN_SET)
	{
		if(flag==0)
		{
//			HAL_Delay(20);
			MC_ProgramSpeedRampMotor1(300/6,10);
			MC_StartMotor1();
			while(HAL_GPIO_ReadPin(Start_Stop_GPIO_Port,Start_Stop_Pin) == GPIO_PIN_SET);

		    return 1;
		}

		else if(flag==1)
		{
//			HAL_Delay(20);
//			MC_ProgramSpeedRampMotor1(600/6,0);

			while(HAL_GPIO_ReadPin(Start_Stop_GPIO_Port,Start_Stop_Pin) == GPIO_PIN_SET);
			return 2;
		}

		else if(flag==2)
		{
//			HAL_Delay(20);
//			MC_ProgramSpeedRampMotor1(900/6,10);
			SPEED=300/6;
			while(HAL_GPIO_ReadPin(Start_Stop_GPIO_Port,Start_Stop_Pin) == GPIO_PIN_SET);
			return 3;
		}
		else if(flag==3)

		{
//			HAL_Delay(20);
//			MC_StopMotor1();
			while(HAL_GPIO_ReadPin(Start_Stop_GPIO_Port,Start_Stop_Pin) == GPIO_PIN_SET);
	        return 2;
		}

//		else if(flag==3)
//		{
//			MC_ProgramSpeedRampMotor1(700/6,10);
//			(void)MC_StartMotor1();
//			return 0;
//		}
	}
	return flag;

/* USER CODE END START_STOP_BTN */
}

int flag = 0;
int mode=0;
int filter_mode=4;
float kp=15;
float ki=0.2;
float current_speed=0;
float dI;



int main(void)
{

  /* USER CODE BEGIN 1 */
	short speed_trans[2]={0};
	int size=10;
	float history_speed[10]={0};
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//  MC_ProgramSpeedRampMotor1(600/6,10);
  a.q = 0;
  a.d = 0;
  I = 0;
//  SPEED=600/6;
  SPEED_expected=700/6;
  /* USER CODE END 2 */
 // MC_ProgramSpeedRampMotor1(700/6,10);
   // MC_StartMotor1();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  int flagb;


	  	flagb = UI_HandleStartStopButton_cb(flag);
	  	flag = flagb;																																																																																																																																																																																																									if(0){
		current_speed = Get_Speed();																																																																																																																																																																																																					}current_speed = MC_GetMecSpeedAverageMotor1();



		//		current_speed = MC_GetMecSpeedAverageMotor1();
		if(current_speed==0){
			current_speed=speed;
		}
		if(flag!=0&&flag!=1){
//			start=clock();
			if(flag==2){
				SPEED=SPEED_expected;
			}
	  		speed=filter_realtime_dsp(filter_mode, current_speed, history_speed, size, speed);
			speed=current_speed;

	  		P = kp*(SPEED - speed);
//	  		if (speed<30){
	  		if (0) {
	  			dI=0;
	  		}else{
	  			arm_sub_f32(&SPEED,&speed,&tmp1,1);
	  			arm_mult_f32(&ki,&tmp1,&dI,1);
//	  			dI=ki*(SPEED - speed);
	  		}
			  I = I + dI;
//			  if(I>20000){
//				  I=20000;
//			  }
			  arm_add_f32(&P,&I,&tmp1,1);
			  a.q = (int)tmp1;
//			  a.q = (int)(P + I);
//			  if(a.q<-10000){
//				  a.q=-10000;
//			  }
			  if(a.q>250){
				  a.q=250;
			  }
//			  if(a.q<-250){
//				  a.q=-250;
//			  }
//			  if(a.q<50){
//				  a.q=50;
//			  }
			  MC_SetCurrentReferenceMotor1(a);
			  MC_StartMotor1();

//			  speed_trans[1]=speed/2;
//			  speed_trans[1]=a.q;
//			  HAL_UART_Transmit(&huart2,(uint8_t*)speed_trans,sizeof(speed_trans),10);
			  HAL_Delay(1);
//			  cnt+=1;
//			  if (cnt==10000){
//				  end=clock();
//				  cof=((float)(end-start))/CLK_TCK;
//			  }


	  	}


//	  SPEED=150;
//	  speed = MC_GetMecSpeedAverageMotor1();
//	  P = 5*(SPEED - speed);
//	  I = I + 0.03*(SPEED - speed);
//	  if(I>2000){
//		  I=2000;
//	  }
//	  a.q = P + I;
//	  MC_SetCurrentReferenceMotor1(a);
////	  MC_StartMotor1();
//	  speed_trans[1]=speed;
//	  HAL_UART_Transmit(&huart2,(uint8_t*)speed_trans,sizeof(speed_trans),10);

//	  	  MC_SetCurrentReferenceMotor1(a);
//	  	  speed[1] = MC_GetMecSpeedAverageMotor1();
//	  	  HAL_UART_Transmit(&huart2,(uint8_t*)speed,sizeof(speed),100);
//	  	speed[1] = MC_GetMecSpeedAverageMotor1()*3;
//	  	HAL_UART_Transmit(&huart2,(uint8_t*)speed,sizeof(speed),100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 3;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 1843200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M1_PWM_EN_U_Pin|M1_PWM_EN_V_Pin|M1_PWM_EN_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_PWM_EN_U_Pin M1_PWM_EN_V_Pin M1_PWM_EN_W_Pin */
  GPIO_InitStruct.Pin = M1_PWM_EN_U_Pin|M1_PWM_EN_V_Pin|M1_PWM_EN_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
