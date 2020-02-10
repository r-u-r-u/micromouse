
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "flash.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

const float Rm = 37.750;//ù‰ñ”¼Œa
const float Rt = 12.0;//ƒ^ƒCƒ„”¼Œa
const float PI = 3.14159265;//‰~Žü—¦
const float T = 0.001;//§ŒäŽüŠú
const float M = 114.58;//‹@‘Ìd—Ê
const float Ratio_gear = 42/8;//ƒMƒA”ä
const float V_source = 3.3;

const float Torque_constant_motor = 1.98*0.001;
const float R_constant_motor = 1.07;
const float Vback_constant_motor = 0.207*0.001;//‹t‹N“d—Í’è”[V/min^-1]
const float Rotation_constant_motor = 4820;//‰ñ“]’è”[min^-1/V]

float time=0.0;

double view_enc1 = 0;
double view_enc2 = 0;
double view_gyro_i2c = 0;
double view_gyro_enc = 0;

double theta=0.0;
double theta_offset=0.0;
enum{LENGTH=4};
uint16_t ADCBuffer[LENGTH];
#define true 1
#define false 0
struct flags{
	short btn1,btn2,start_tim;
} flg;
struct position{
	float x,y;
} pos;
struct parcel{
	int x,y;
	short n,s,e,w;
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void user_tim1_ch2_pwm_setvalue(float value);
void user_tim5_ch3_pwm_setvalue(float value);
void user_tim5_ch4_pwm_setvalue(float value);
void user_tim8_ch3_pwm_setvalue(float value);
void user_tim8_ch4_pwm_setvalue(float value);
void user_tim4_ch1_pwm_setvalue(float value);
void user_tim4_ch2_pwm_setvalue(float value);
void user_tim4_ch3_pwm_setvalue(float value);
void user_tim4_ch4_pwm_setvalue(float value);
void user_tim4_pwm_setvalue(float value);
void user_tim2_qei_setvalue(int32_t);
void user_tim3_qei_setvalue(int32_t);
int32_t user_tim2_qei_getvalue();
int32_t user_tim3_qei_getvalue();
void user_i2c2_gyro_set_init();
float user_i2c2_gyro_getvalue();
double user_enc_gyro_getvalue(float right,float left);
void user_motor1_duty(float target);
void user_motor2_duty(float target);
float user_motor_foward(float target,float encval);
void user_motor1_control(float target,float encval);
void user_motor2_control(float target,float encval);
void user_motor1_stop();
void user_motor2_stop();
float user_motor1_qei_getvalue();
float user_motor2_qei_getvalue();
float user_pid_motor1_speed(float target,float current);
float user_pid_motor2_speed(float target,float current);
float user_pid_motor1_position(float target,float current);
float user_pid_motor2_position(float target,float current);
float user_pid_theta(float target,float current);

void kinematics(float v,float rho,float *left,float *right);
void user_machine_control_speed(float target_v,float target_w);
GPIO_PinState user_button_1();
GPIO_PinState user_button_2();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void user_tim6_interrupt(){
	time += 0.001;
	float encval1=user_motor1_qei_getvalue();//left
	float encval2=user_motor2_qei_getvalue();//right
	view_enc1 = encval1;
	view_enc2 = encval2;
	view_gyro_i2c = user_i2c2_gyro_getvalue();
	view_gyro_enc = user_enc_gyro_getvalue(encval2,encval1);
	theta += view_gyro_i2c;



	float control_val = theta*500 + view_gyro_i2c*Rt;

	user_motor1_control(control_val,encval1);
	user_motor2_control(-control_val,encval2);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
}
void user_tim7_interrupt(){
	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  user_tim1_ch2_pwm_setvalue(0.0);
  user_tim5_ch3_pwm_setvalue(0.0);
  user_tim5_ch4_pwm_setvalue(0.0);
  user_tim8_ch3_pwm_setvalue(0.0);
  user_tim8_ch4_pwm_setvalue(0.0);
  user_tim4_pwm_setvalue(0.0);
  user_tim2_qei_setvalue(0);
  user_tim3_qei_setvalue(0);

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  user_i2c2_gyro_set_init();

  memset(ADCBuffer,0,sizeof(ADCBuffer));
  HAL_ADC_Start_DMA(&hadc1,ADCBuffer,LENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(user_button_1()) flg.btn1=true;
	  if(user_button_2()) flg.btn2=true;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void user_tim1_ch2_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((4000-1)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}
void user_tim5_ch3_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((800-1)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}
void user_tim5_ch4_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((800-1)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}
void user_tim8_ch3_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((800-1)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}
void user_tim8_ch4_pwm_setvalue(float value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = (uint32_t)((800-1)*value);
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}
void user_tim4_ch1_pwm_setvalue(float value){
	   TIM_OC_InitTypeDef sConfigOC;
	   sConfigOC.OCMode = TIM_OCMODE_PWM1;
	   sConfigOC.Pulse = (uint32_t)((80000-1)*value);
	   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	   HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}
void user_tim4_ch2_pwm_setvalue(float value){
	   TIM_OC_InitTypeDef sConfigOC;
	   sConfigOC.OCMode = TIM_OCMODE_PWM1;
	   sConfigOC.Pulse = (uint32_t)((80000-1)*value);
	   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	   HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
	   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
void user_tim4_ch3_pwm_setvalue(float value){
	   TIM_OC_InitTypeDef sConfigOC;
	   sConfigOC.OCMode = TIM_OCMODE_PWM1;
	   sConfigOC.Pulse = (uint32_t)((80000-1)*value);
	   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	   HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
	   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
void user_tim4_ch4_pwm_setvalue(float value){
	   TIM_OC_InitTypeDef sConfigOC;
	   sConfigOC.OCMode = TIM_OCMODE_PWM1;
	   sConfigOC.Pulse = (uint32_t)((80000-1)*value);
	   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	   HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
	   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}
void user_tim4_pwm_setvalue(float value){
	user_tim4_ch1_pwm_setvalue(value);
	user_tim4_ch2_pwm_setvalue(value);
	user_tim4_ch3_pwm_setvalue(value);
	user_tim4_ch4_pwm_setvalue(value);
}
void user_tim2_qei_setvalue(int32_t val){
	TIM2->CNT = val;
}
void user_tim3_qei_setvalue(int32_t val){
	TIM3->CNT = val;
}
int32_t user_tim2_qei_getvalue(){
	return (int32_t)TIM2->CNT;
}
int32_t user_tim3_qei_getvalue(){
	return (int32_t)TIM3->CNT;
}
double user_enc_gyro_getvalue(float right,float left){
	return -((PI*Rt*(right-left))/(1024*4*Ratio_gear*Rm));
}
float user_i2c2_gyro_getvalue(){
	union Gyro_Data{
		uint8_t data_splited[2];
		int16_t data_composed;
	} dat;
	HAL_I2C_Mem_Read(&hi2c2,0xD0,0x47,I2C_MEMADD_SIZE_8BIT,&dat.data_splited[1],1,10);
	HAL_I2C_Mem_Read(&hi2c2,0xD0,0x48,I2C_MEMADD_SIZE_8BIT,&dat.data_splited[0],1,10);
	return (float)(dat.data_composed*1.0/16400*PI/180);
}
void user_i2c2_gyro_set_init(){
	uint8_t read=0;
	uint8_t address=0x75;//WHO_AM_I
	uint8_t config=0x00;
	uint8_t set[]={0x1A,0x12};
	uint8_t dat=0x00;
	HAL_I2C_Mem_Read(&hi2c2,0xD0/*device ID*/,0x75/*WHO_AM_I*/,I2C_MEMADD_SIZE_8BIT/*Memory Address Size*/,&read/*Data Buffer*/,1,10);
	if( read == 0x68 ){
		//wakeup sensor
		HAL_I2C_Mem_Write(&hi2c2,0xD0,0x6B,I2C_MEMADD_SIZE_8BIT,&dat,1,10);
		HAL_I2C_Mem_Read(&hi2c2 ,0xD0,0x6B,I2C_MEMADD_SIZE_8BIT,&read,1,10);
		//configure sensor
		config=0x00;
		HAL_I2C_Mem_Write(&hi2c2,0xD0,0x1a,I2C_MEMADD_SIZE_8BIT,&config,1,10);
		config=0x10;
		HAL_I2C_Mem_Write(&hi2c2,0xD0,0x1b,I2C_MEMADD_SIZE_8BIT,&config,1,10);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	}else{/*read == 0x68*/

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	}
}
void user_motor1_duty(float target){
	if(target>0){
		user_tim5_ch3_pwm_setvalue(target);
		user_tim5_ch4_pwm_setvalue(0.0);
	}else
	if(target<0){
		user_tim5_ch3_pwm_setvalue(0.0);
		user_tim5_ch4_pwm_setvalue(-target);
	}else{
		user_tim5_ch3_pwm_setvalue(0.0);
		user_tim5_ch4_pwm_setvalue(0.0);
	}
}
void user_motor2_duty(float target){
	if(target>0){
		user_tim8_ch3_pwm_setvalue(target);
		user_tim8_ch4_pwm_setvalue(0.0);
	}else
	if(target<0){
		user_tim8_ch3_pwm_setvalue(0.0);
		user_tim8_ch4_pwm_setvalue(-target);
	}else{
		user_tim8_ch3_pwm_setvalue(0.0);
		user_tim8_ch4_pwm_setvalue(0.0);
	}
}
void user_motor1_stop(){
	user_motor1_duty(0);
}
void user_motor2_stop(){
	user_motor2_duty(0);
}
void user_motor1_control(float target,float encval){
	user_motor1_duty(user_motor_foward(-target,encval)+user_pid_motor1_position(-target,encval));
}
void user_motor2_control(float target,float encval){
	user_motor2_duty(user_motor_foward(-target,encval)+user_pid_motor2_position(-target,encval));
}
float user_motor_foward(float target,float encval){
	float ff_acceleration = target;

	float torque_target = M * ff_acceleration * Rt/2;
	float torque_motor = torque_target/Ratio_gear;
	float i_motor = torque_motor/Torque_constant_motor;
	float v_back_motor = Vback_constant_motor*(encval*60.0*1000)*0;
//	float v_motor = R_constant_motor*i_motor + v_back_motor;
	float v_motor = (target*60.0)/Rotation_constant_motor + v_back_motor;
	float duty = v_motor/V_source;

	return duty;
}

float user_motor1_qei_getvalue(){
	float value = (float)(user_tim2_qei_getvalue()-65535/2);
	user_tim2_qei_setvalue(65535/2);
	return value;
}
float user_motor2_qei_getvalue(){
	float value = -(float)(user_tim3_qei_getvalue()-65535/2);
	user_tim3_qei_setvalue(65535/2);
	return value;
}
float user_pid_motor1_speed(float target,float current){
	static float P=0.0,I=0.0,D=0.0;
	static float diff[3]={0.0};
	static float u;
	diff[2]=diff[1];
	diff[1]=diff[0];
	diff[0]=target-current;
	u += P*(diff[0]-diff[1])+ I*diff[0]+ D*(diff[0]-2*diff[1]+diff[2]);
	return u;
}
float user_pid_motor2_speed(float target,float current){
	static float P=0.0,I=0.0,D=0.0;
	static float diff[3]={0.0};
	static float u;
	diff[2]=diff[1];
	diff[1]=diff[0];
	diff[0]=target-current;
	u += P*(diff[0]-diff[1])+ I*diff[0]+ D*(diff[0]-2*diff[1]+diff[2]);
	return u;
}
float user_pid_motor1_position(float target,float current){
	float duty=0.0;
	static float P=0.0,I=0.0,D=0.0;
	float diff[3]={0.0};
	diff[2]=(target-current)-diff[0];
	diff[1]+=(target-current);
	diff[0]=target-current;
	duty = P*diff[0]+ I*diff[1]+ D*diff[2];
	return duty;
}
float user_pid_motor2_position(float target,float current){
	float duty=0.0;
	static float P=0.0,I=0.0,D=0.0;
	float diff[3]={0.0};
	diff[2]=(target-current)-diff[0];
	diff[1]+=(target-current);
	diff[0]=target-current;
	duty = P*diff[0]+ I*diff[1]+ D*diff[2];
	return duty;
}
void user_machine_control_speed(float target_v,float target_w){
}
float user_pid_theta(float target,float current){
}
void kinematics(float v,float rho,float *left,float *right){
	float w = 0.0;
	if(rho == 0.0){
		w = v;
	}
	else if(v=0.0){
		w = rho;
	}else{
		w = v/rho;
	}
	*right = (rho + Rm)*w;
	*left = (rho - Rm)*w;
}
GPIO_PinState user_button_1(){
	return !HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
}
GPIO_PinState user_button_2(){
	return !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
