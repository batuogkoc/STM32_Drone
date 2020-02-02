
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "MadgwickAHRS.h"
#include <math.h>
#include "string.h"
#include "MY_NRF24.h"
#include "string.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//reciever channel allocations
#define rollIn_ch ch[3]
#define pitchIn_ch ch[1]
#define throttleIn_ch ch[2]
#define yawIn_ch ch[0]

//control endpoints
#define roll_pitch_bankAng_max 30
#define yaw_max_degPsec 200

//hardware peripheral labeling
#define MOTOR_TIMER htim3


_Bool new_data = 0;
_Bool offsetting = 1;
//I2C Variables
uint8_t txBuffer[2];
uint8_t rxBuffer[14];
int16_t rawData[7];

//Mpu data storage variables
float accX, accY, accZ, temp, gyroX, gyroY, gyroZ, gyroX_rad, gyroY_rad, gyroZ_rad; //raw usable data
float accX_raw, accY_raw, accZ_raw, gyroX_raw, gyroY_raw, gyroZ_raw;
float roll, pitch, yaw;
float x_degrees,y_degrees,z_degrees;
float w,x,y,z;

//Mpu settings variables
/*float gyro_sensitivity_scale_factor = 131.0f; //±250deg/s
float gyro_sensitivity_scale_factor = 65.5f; //±500deg/s
float gyro_sensitivity_scale_factor = 32.8f; //±1000deg/s*/
float gyro_sensitivity_scale_factor = 16.4f; //±2000deg/s

/*float acc_sensitivity_scale_factor = 16384.0f; //±2g
float acc_sensitivity_scale_factor = 8192.0f; //±4g
float acc_sensitivity_scale_factor = 4096.0f; //±8g*/
float acc_sensitivity_scale_factor = 2048.0f; //±16g

//Mpu tweaking variables
int32_t gyroX_Offset = 0;
int32_t gyroY_Offset = 0;
int32_t gyroZ_Offset = 0;
float accX_Offset = -45.83873227210463;
float accY_Offset = 315.6486942077451;
float accZ_Offset = -70.77163484576726;
float accX_Scale = 0.997142840160418212890625;
float accY_Scale = 1.0071514293393857421875;
float accZ_Scale = 1.009796179997785986328125;

//Mpu timing variables
unsigned long reading_elapsed_time = 0;
unsigned long reading_last_time = 0;
uint32_t samples = 0;
uint32_t reading_count = 0;
uint32_t this_time = 0;

//Timekeeping Variables
uint16_t loop_freq = 250; //250 hz (desired)
uint32_t loop1_last = 0;
uint32_t loop2_last = 0;
uint32_t loop3_last = 0;
uint32_t loop4_last = 0;
uint32_t loop5_last = 0;
uint32_t time_stamps[8];
//uint32_t loop3_last = 0;
uint32_t system_clock = 0;
uint32_t loop_count = 500;

//Maths variables
const float rad_to_deg = 57.2957795f;
const float deg_to_rad = 1.0f / rad_to_deg;

//nrf24 variables(imported from "NRF24 Tutorial RX" with no changes!)
uint64_t RXpipe_address = 0x11223344AA;
char rx_data[40];
char ack_data[32] = "I have recieved your data";
_Bool nrf24_available = 0;

//ppm reciever input variables
uint32_t ch[8];
uint32_t ch_count = 9;
const uint16_t reciever_low = 996, reciever_mid = 1502, reciever_high = 2016;

//general reciever variables
double reciever_beta = 0.05;//WIP
double rollIn, pitchIn, yawIn, throttleIn;
uint32_t last_signal;

//Pid variables (imported from "Drone Test")
uint32_t pitch_current_time,pitch_last_time;
uint32_t roll_current_time,roll_last_time;
uint32_t yaw_current_time,yaw_last_time;
double pitch_elapsed_time, roll_elapsed_time, yaw_elapsed_time;
double pitch_pid_error, pitch_error, pitch_last_error = 0, pitch_p_error, pitch_i_error, pitch_d_error;
double roll_pid_error, roll_error, roll_last_error = 0, roll_p_error, roll_i_error, roll_d_error;
double yaw_pid_error, yaw_error, yaw_last_error = 0, yaw_p_error, yaw_i_error, yaw_d_error;
#define pM 0.5 //0.5
double pKp = 1.5*pM, pKi = 1.5*pM, pKd = 0.5*pM;//1.5 1.5  0.5 (tweak i) (throttle and sticks a bit twitchy)
double rKp = 1.5*pM, rKi = 1.5*pM, rKd = 0.5*pM;
double yKp = 1*pM, yKi = 0.5*pM, yKd = 0*pM;//1 0.5 0 (tweak i)
double tpa_pid_p = 0.14; //reduces pid_p value as throttle increases (0.14)
double pitch_pid_output, roll_pid_output, yaw_pid_output;
#define pid_i_range 10 //10

//Motor variables (ported from "Drone Test")
double motorTR, motorBR, motorBL, motorTL;
const double servo_low = 18000;
const double servo_high = 36000;
bool failsafe = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
//PUTCHAR_PROTOTYPE  {
//	/*for(int i = 0; i<400; i++){
//		if(CDC_Transmit_FS((uint8_t*)&ch, 1) == USBD_OK)
//		break;
//	}*/
//	while(!(CDC_Transmit_FS((uint8_t*)&ch, 1) == USBD_OK));
//	//while(!(CDC_Transmit_FS((uint8_t*)&ch, 1) == USBD_BUSY));
//	//CDC_Transmit_FS((uint8_t*)&ch, 1);
//	return ch;
//}
double map_v1(double input, double input_low, double input_high, double output_low, double output_high){
	/*if(input<input_low){
		input = input_low;
	}
	else if(input>input_high){
		input = input_high;
	}*/
	return (((input-input_low)/(input_high-input_low))*(output_high-output_low))+output_low;
}
double map_with_midpoint(double input, double input_low, double input_mid, double input_high, double output_low, double output_high){
	double output_mid = (output_high + output_low)/2;
	if(input < input_mid)
		return map_v1(input, input_low, input_mid, output_low, output_mid);
	else if(input_mid <= input )
		return map_v1(input, input_mid, input_high, output_mid, output_high);
}
uint8_t mpu_register_read(uint8_t address){
	uint8_t txBuf[1] = {address};
	uint8_t status_w = HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, txBuf, 1, 100);//set pointer
	uint8_t status_r = HAL_I2C_Master_Receive(&hi2c1, 0x68<<1, rxBuffer, 1, 100);
	printf("StatusW: %i StatusR: %i\r\n", status_w, status_r);
	return rxBuffer[0];
}
void mpu_register_set(uint8_t address, uint8_t data){
	txBuffer[0] = address;
	txBuffer[1] = data;
	uint8_t status_w = HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, txBuffer, 2, 1000);
	printf("Status_W: %i Reg_cont: %i\n", status_w, mpu_register_read(address));
}

uint8_t mpu_register_set_new(uint8_t address, uint8_t data, uint8_t tries){
	uint8_t data_read;
	for(int i = 0; i<4; i++){
		txBuffer[0] = address;
		txBuffer[1] = data;
		if(HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, txBuffer, 2, 1000) != HAL_OK){
			printf("i2c tx error\r\n");
		}
		data_read = mpu_register_read(address);
		if(data_read == data){
			return HAL_OK;
		}
	}
	if(data_read == data){
			return HAL_ERROR;
	}
}


void mpu_read(void){

	txBuffer[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, txBuffer, 1, 100);//set pointer
	HAL_I2C_Master_Receive(&hi2c1, 0x68<<1, rxBuffer, 14, 100);//read all data registers
	//do timekeeping
	for(int i = 0; i<7; i++){
	rawData[i] = rxBuffer[i*2]<<8 | rxBuffer[(i*2)+1]; //combine bytes into raw measurements
	}
	//calculate readable raw data
	accX_raw = rawData[0];
	accY_raw = rawData[1];
	accZ_raw = rawData[2];
	gyroX_raw = rawData[4];
	gyroY_raw = rawData[5];
	gyroZ_raw = rawData[6];
	if(samples < 500) {
			samples++;
			offsetting = 1;
			return;
		} 
		else if(samples < 1500) {
			gyroX_Offset += (int32_t)gyroX_raw;
			gyroY_Offset += (int32_t)gyroY_raw;
			gyroZ_Offset += (int32_t)gyroZ_raw;
			samples++;
			offsetting = 1;
			return;
		} 
		else{
			/*gyroX_Offset /= 1000;
			gyroY_Offset /= 1000;
			gyroZ_Offset /= 1000;*/
		} 
		offsetting = 0;
		gyroX_raw -= gyroX_Offset/1000;
		gyroY_raw -= gyroY_Offset/1000;
		gyroZ_raw -= gyroZ_Offset/1000;
		/*gyroX_raw -= gyroX_Offset;
		gyroY_raw -= gyroY_Offset;
		gyroZ_raw -= gyroZ_Offset;*/
		accX_raw = (accX_raw + accX_Offset)*accX_Scale;
		accY_raw = (accY_raw + accY_Offset)*accY_Scale;
		accZ_raw = (accZ_raw + accZ_Offset)*accZ_Scale;
		
		
		accX = ((float)accX_raw)/acc_sensitivity_scale_factor;
		accY = ((float)accY_raw)/acc_sensitivity_scale_factor;
		accZ = ((float)accZ_raw)/acc_sensitivity_scale_factor;
		temp = (((float)rawData[3])/340) + 36.53;
		gyroX = (((float)(gyroX_raw))/gyro_sensitivity_scale_factor);
		gyroY = (((float)(gyroY_raw))/gyro_sensitivity_scale_factor);
		gyroZ = (((float)(gyroZ_raw))/gyro_sensitivity_scale_factor);
		
		gyroX_rad = gyroX * deg_to_rad;
		gyroY_rad = gyroY * deg_to_rad;
		gyroZ_rad = gyroZ * deg_to_rad;
		
}	
uint32_t microseconds(void){
	return system_clock*1000 + ((SysTick->LOAD - SysTick->VAL)/72); //SYSTICK IS A DOWN COUNTER; CHANGE!!!!
}

void mpu_setup(void){
	HAL_Delay(100);
	uint8_t error = mpu_register_set_new(0x6B,0x0, 2); //Power Management 1
	uint8_t mpu_status = HAL_I2C_IsDeviceReady(&hi2c1, 0x68<<1, 5, 50);
	if(mpu_status == HAL_OK){
		error |= mpu_register_set_new(0x19,15, 2); //Sample rate divider
		error |= mpu_register_set_new(0x1A,0x0, 2); //DLPF Config
	
	
  	error |= mpu_register_set_new(0x1B,3<<3, 2); //Gyro Config (2000 d)
  	error |= mpu_register_set_new(0x1C,3<<3, 2); //Acc Config	(16 g)
	
	
    error |= mpu_register_set_new(56,1, 2); //Interrupt Config
	}
	else{
		printf("\nError at connection: %i\n", mpu_status);
		failsafe = 1;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_NVIC_SystemReset();
	}
	if(error != HAL_OK){
		printf("\nError at setup: %i\n", error);
		failsafe = 1;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_NVIC_SystemReset();
	}
	else{
		printf("Setup successful!\n");
	}

}
void QuatToEuler(void){
	float sqw;
	float sqx;
	float sqy;
	float sqz;

	float rotxrad;
	float rotyrad;
	float rotzrad;

	sqw = q0* q0;
	sqx = q1* q1;
	sqy = q2* q2;
	sqz = q3* q3;
	
	rotxrad = (float) atan2(2.0 * (q2* q3 + q1* q0 ) , ( -sqx - sqy + sqz + sqw ));
	rotyrad = (float) asin(-2.0 * (q1* q3 - q2* q0 ));
	rotzrad = (float) atan2(2.0 * (q1* q2 + q3* q0 ) , (sqx - sqy - sqz + sqw ));

	x_degrees = rotxrad * rad_to_deg;
	y_degrees = rotyrad * rad_to_deg;
	z_degrees = rotzrad * rad_to_deg;
	roll = -x_degrees;
	pitch = y_degrees;
	yaw = -z_degrees;
	return;
}
void nrf24_setup(void){
  NRF24_begin(CE_GPIO_Port,CSN_Pin,CE_Pin,hspi2);
	nrf24_DebugUART_Init(huart1);
	
	NRF24_openReadingPipe(1,RXpipe_address);
	NRF24_setAutoAck(true);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	NRF24_setChannel(42);
	NRF24_setPayloadSize(32);
	NRF24_setDataRate(RF24_250KBPS);
	NRF24_startListening();
	printRadioSettings();
}
void nrf24_test(void){
	if(nrf24_available){
		nrf24_available = 0;
		NRF24_read(rx_data,32);
		NRF24_writeAckPayload(1,ack_data,32);
		
		
		rx_data[32] = '\r'; rx_data[33] = '\n';
		HAL_UART_Transmit(&huart1, (uint8_t*)rx_data,34,10);
		if(rx_data[0] == '0')
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 1);
		else if(rx_data[0] == '1')
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 0);
	}
}
void reciever_print(void){
	for(int i = 0; i<8; i++){
		  printf("CHANNEL %i: %i ", i+1,ch[i]);
		}
  printf("\n");
}
void pitchPidCompute(void){
  pitch_last_error = pitch_error;
  pitch_error = pitch-pitchIn;
  pitch_last_time = pitch_current_time;
  pitch_current_time = microseconds();
  pitch_elapsed_time = (pitch_current_time-pitch_last_time)/ 1000000.0; //!! eliminate the division by the microseconds period if problems arise !!
  pitch_pid_error = 0;
  pitch_p_error = pKp*pitch_error*map_v1(throttleIn, 0, 180, 1+tpa_pid_p, 1-tpa_pid_p);
  if((pid_i_range>pitch_error)&&(pitch_error>-pid_i_range)){
    pitch_i_error += pKi*(pitch_error * pitch_elapsed_time);
  }
	else{
		pitch_i_error = 0;
	}
	if(failsafe == 1 || throttleIn < 10){
		pitch_i_error = 0;
	}
  pitch_d_error = (pKd*((pitch_error-pitch_last_error)/pitch_elapsed_time)); 
  pitch_pid_output = pitch_p_error + pitch_i_error + pitch_d_error;
	
  /*if(pitch_pid_output > 30)
		pitch_pid_output = 30;
	else if(pitch_pid_output < -30)
		pitch_pid_output = -30;*/
}
void rollPidCompute(void){
  roll_last_error = roll_error;
  roll_error = roll-rollIn;
  roll_last_time = roll_current_time;
  roll_current_time = microseconds();
	roll_elapsed_time = (roll_current_time-roll_last_time)/1000000.0;//!! eliminate the division by the microseconds period if problems arise !!
  roll_pid_error = 0;
  roll_p_error = rKp*roll_error*map_v1(throttleIn, 0, 180, 1+tpa_pid_p, 1-tpa_pid_p);
  if((pid_i_range>roll_error && roll_error>-pid_i_range)){
    roll_i_error += rKi*(roll_error * roll_elapsed_time);
  }
	else{
		roll_i_error = 0;
	}
	if(failsafe == 1 || throttleIn < 10){
		roll_i_error = 0;
	}
  roll_d_error = (rKd*((roll_error-roll_last_error)/roll_elapsed_time)); 
  roll_pid_output = roll_p_error + roll_i_error + roll_d_error;
	
  /*if(roll_pid_output > 30)
		roll_pid_output = 30;
	else if(roll_pid_output < -30)
		roll_pid_output = -30;*/
}
void yawPidCompute(){
  yaw_last_error = yaw_error;
  yaw_error = gyroZ - yawIn;
  yaw_last_time = yaw_current_time;
  yaw_current_time = microseconds();
  yaw_elapsed_time = (yaw_current_time-yaw_last_time)/1000000.0;//!! eliminate the division by the microseconds period if problems arise !!
  yaw_pid_error = 0;
  yaw_p_error = yKp*yaw_error*map_v1(throttleIn, 0, 180, 1+tpa_pid_p, 1-tpa_pid_p);
  if(pid_i_range>yaw_error && yaw_error>-pid_i_range){
    yaw_i_error += yKi*(yaw_error  * yaw_elapsed_time);
  }
	else{
		yaw_i_error = 0;
	}
	if(failsafe == 1 || throttleIn < 10){
		yaw_i_error = 0;
	}
  yaw_d_error = (yKd*((yaw_error-yaw_last_error)/yaw_elapsed_time)); 
  yaw_pid_output = yaw_p_error + yaw_i_error + yaw_d_error;
  
	/*if(yaw_pid_output > 30)
		yaw_pid_output = 30;
	else if(yaw_pid_output < -30)
		yaw_pid_output = -30;*/
}
void motor_signal_set(bool failsf){
	motorTR = throttleIn;
	motorBR = throttleIn;
	motorBL = throttleIn; 
	motorTL = throttleIn;
	
	
	//pitch(fix clip protect)
	motorTR -= pitch_pid_output;
	if(motorTR<0){
		motorBR += 0 - motorTR;
		motorTR = 0;
	}
	else if(180<motorTR){
		motorBR += 180 - motorTR;
		motorTR = 180;
	}
	
	motorBR += pitch_pid_output;
	if(motorBR<0){
		motorTR += 0 - motorBR;
		motorBR = 0;
	}
	else if(180<motorBR){
		motorTR += 180 - motorBR;
		motorBR = 180;
	}
	
	motorBL += pitch_pid_output; 
	if(motorBL<0){
		motorTL += 0 - motorBL;
		motorBL = 0;
	}
	else if(180<motorBL){
		motorTL += 180 - motorBL;
		motorBL = 180;
	}
	
	motorTL -= pitch_pid_output;
	if(motorTL<0){
		motorBL += 0 - motorTL;
		motorTL = 0;
	}
	else if(180<motorTL){
		motorBL += 180 - motorTL;
		motorTL = 180;
	}
	
	
	
	
	//roll(fix clip protect)
	motorTR += roll_pid_output;
	if(motorTR<0){
		motorTL += 0 - motorTR;
		motorTR = 0;
	}
	else if(180<motorTR){
		motorTL += 180 - motorTR;
		motorTR = 180;
	}
	
	motorTL -= roll_pid_output;
	if(motorTL<0){
		motorTR += 0 - motorTL;
		motorTL = 0;
	}
	else if(180<motorTL){
		motorTR += 180 - motorTL;
		motorTL = 180;
	}
	
	motorBR += roll_pid_output;
	if(motorBR<0){
		motorBL += 0 - motorBR;
		motorBR = 0;
	}
	else if(180<motorBR){
		motorBL += 180 - motorBR;
		motorBR = 180;
	}
	
	motorBL -= roll_pid_output; 
	if(motorBL<0){
		motorBR += 0 - motorBL;
		motorBL = 0;
	}
	else if(180<motorBL){
		motorBR += 180 - motorBL;
		motorBL = 180;
	}
	
	
	
	/*motorTR += yawIn;
	motorBR -= yawIn;
	motorBL += yawIn; 
	motorTL -= yawIn;*/

	motorTR += yaw_pid_output;
	motorBR -= yaw_pid_output;
	motorBL += yaw_pid_output; 
	motorTL -= yaw_pid_output;
	
	motorTL = map_v1(motorTL,0,180,servo_low,servo_high);
	motorTR = map_v1(motorTR,0,180,servo_low,servo_high);
	motorBL = map_v1(motorBL,0,180,servo_low,servo_high);
	motorBR = map_v1(motorBR,0,180,servo_low,servo_high);
	//clipping
  if(motorTL<servo_low){
		motorTL = servo_low;
	}
	else if(motorTL>servo_high){
		motorTL = servo_high;
	}
	if(motorTR<servo_low){
		motorTR = servo_low;
	}
	else if(motorTR>servo_high){
		motorTR = servo_high;
	}	
	if(motorBL<servo_low){
		motorBL = servo_low;
	}
	else if(motorBL>servo_high){
		motorBL = servo_high;
	}	
	if(motorBR<servo_low){
		motorBR = servo_low;
	}
	else if(motorBR>servo_high){
		motorBR = servo_high;
	}
	/*motorTR = servo_low;
	motorBR = servo_high;
	motorBL = servo_low; 
	motorTL = servo_high;*/
	
	//pwm signal set
	if(failsf || (throttleIn < 10)){
		motorTR = servo_low;
		motorBR = servo_low;
		motorBL = servo_low; 
		motorTL = servo_low;
	}
	__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, motorTR);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_2, motorBR);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_3, motorBL);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_4, motorTL);
//	HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_4);
//	HAL_TIM_OnePulse_Start(&MOTOR_TIMER,TIM_CHANNEL_1);
//	HAL_TIM_OnePulse_Start(&MOTOR_TIMER,TIM_CHANNEL_2);
//	HAL_TIM_OnePulse_Start(&MOTOR_TIMER,TIM_CHANNEL_3);
//	HAL_TIM_OnePulse_Start(&MOTOR_TIMER,TIM_CHANNEL_4);
}
void map_reciever_inputs(){	
	
	//WIP Low pass filter
	double rollIn_raw     = map_with_midpoint((double)rollIn_ch, reciever_low, reciever_mid, reciever_high, -roll_pitch_bankAng_max,  roll_pitch_bankAng_max);
	double pitchIn_raw    = -map_with_midpoint((double)pitchIn_ch, reciever_low, reciever_mid, reciever_high, -roll_pitch_bankAng_max,  roll_pitch_bankAng_max);
	double yawIn_raw      = -map_with_midpoint((double)yawIn_ch, reciever_low, reciever_mid, reciever_high, -yaw_max_degPsec,  yaw_max_degPsec);
	double throttleIn_raw = map_with_midpoint((double)throttleIn_ch, reciever_low, reciever_mid, reciever_high,   0.0f, 180.0f);
	
	rollIn = rollIn - (reciever_beta * (rollIn - rollIn_raw));
	pitchIn = pitchIn - (reciever_beta * (pitchIn - pitchIn_raw));
	yawIn = yawIn - (reciever_beta * (yawIn - yawIn_raw));
	throttleIn = throttleIn - (reciever_beta * (throttleIn - throttleIn_raw));
	
	
	 /*rollIn     = map_with_midpoint((double)rollIn_ch, reciever_low, reciever_mid, reciever_high, -roll_pitch_bankAng_max,  roll_pitch_bankAng_max);
	 pitchIn    = map_with_midpoint((double)pitchIn_ch, reciever_low, reciever_mid, reciever_high, -roll_pitch_bankAng_max,  roll_pitch_bankAng_max);
	 yawIn      = map_with_midpoint((double)yawIn_ch, reciever_low, reciever_mid, reciever_high, -yaw_max_degPsec,  yaw_max_degPsec);
	 throttleIn = map_with_midpoint((double)throttleIn_ch, reciever_low, reciever_mid, reciever_high,   0.0f, 180.0f);*/
}
void time_stamps_print(uint16_t last_timestamp){
	printf("TOTAL TIME: %i\n", (time_stamps[last_timestamp] - time_stamps[0]));
	for(int i = 1; i<last_timestamp; i++){
		printf("TIMESTAMP %i:, %i\n", i, (time_stamps[i] - time_stamps[i-1]));
	}
	printf("\n");
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	
	  SysTick_Config(SystemCoreClock/1000);//set systick up
//		mpu_register_set_new(0x19,15); //Sample rate divider
//		while(true)
//			printf("%i ", mpu_register_read(117));		
		//timer start
		HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&MOTOR_TIMER,TIM_CHANNEL_4);
		
		//nrf24_setup();
		HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1); //ppm recieve start
		loop1_last = microseconds();
		loop2_last = microseconds();
		mpu_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(new_data){
			time_stamps[0] = microseconds();
			
			HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,1);
			new_data = 0;
			mpu_read(); //optimisable
			
			time_stamps[1] = microseconds();
			
			if(offsetting == 0){
				MadgwickAHRSupdateIMU(gyroX_rad, gyroY_rad, gyroZ_rad, accX, accY, accZ);//unoptimisable
			}
//			this_time = microseconds();
//			reading_elapsed_time = this_time - reading_last_time;
//			reading_last_time = this_time;
			QuatToEuler();//partly optimisable
			
			time_stamps[2] = microseconds();
			
			map_reciever_inputs();
			
			rollPidCompute();
			pitchPidCompute();
		  yawPidCompute();
			
			time_stamps[3] = microseconds();
			
			loop3_last = microseconds();
			
			if(loop_count>450 && loop_count<510 && (microseconds()-last_signal)<100000){
				failsafe = 0;
				motor_signal_set(failsafe);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
			}
			else{
				failsafe = 1;
				motor_signal_set(failsafe);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
				//HAL_NVIC_SystemReset();
			}
			reading_count++;
			//printf("%f\n", gyroY_rad*rad_to_deg);
			time_stamps[4] = microseconds();
			HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,0);
		}
		if(system_clock - loop1_last> 1000){
			loop_count = reading_count;
		  loop1_last = system_clock;
			reading_count = 0;
		}
		
		/*if(1/((microseconds() - loop3_last)/1000000)> 200){
		  //Add motor set to here igf you want to run slower than loop freq
		}*/
		
		if(system_clock - loop2_last> 250){
			loop2_last = system_clock;
			uint32_t start_a = microseconds();
			
			//CDC_Transmit_FS(pack, strlen((char *)pack));
			//printf("Hello\n");
//			printf("%f\n",x_degrees);
//			printf("Y Deg : %f\n",y_degrees);
//			printf("Z Deg : %f\n",z_degrees);
//			printf("ROLL : %f\n",roll);
//			printf("PITCH : %f\n",pitch);
//			printf("YAW : %f\n",yaw);
//			printf("TL%f\n", motorTL);
//			printf("TR%f\n", motorTR);
//			printf("BL%f\n", motorBL);
//			printf("%f\n", motorBR);
//			printf("%f\n", ((motorBR + motorBL + motorTL + motorTR)/4)/(map_with_midpoint(throttleIn_ch, reciever_low, reciever_mid, reciever_high, 18000, 36000)));
//			printf("ROLL IN  : %f\n",rollIn);
//			printf("PITCH IN : %f\n",pitchIn);
//			printf("YAW IN   : %f\n",yawIn);
//			printf("THRO IN   : %f\n",throttleIn);
			//time_stamps_print(4);
//			reciever_print();
//			printf("X%f\n",accX_raw);
//			printf("Y%f\n",accY_raw);
//			printf("Z%f\n",accZ_raw);
//			printf("GYX : %i\n",gyroX_raw);
//			printf("GYY : %i\n",gyroY_raw);
//			printf("GYZ : %i\n",gyroZ_raw);
//			printf("GYX : %i\n",gyroX_Offset);
//			printf("GYY : %i\n",gyroY_Offset);
//			printf("GYZ : %i\n",gyroZ_Offset);
//			reciever_print();
//			printf("L count m: %i\n",loop_count);
				//printf("Print time: %i\n",(microseconds()-start_a));
		}
		//nrf24_test();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 36100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_Interrupt_Pin MPU_Interrupt_Pin */
  GPIO_InitStruct.Pin = NRF_Interrupt_Pin|MPU_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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
