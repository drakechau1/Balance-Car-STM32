/*
 * MotorControl.c
 *
 *  Created on: Nov 19, 2020
 *      Author: duc
 */


#include "MotorControl.h"

TIM_HandleTypeDef htim1;

void initMotor(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}
void stopMotor(void)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim1,ENA_Pin);
	HAL_TIM_PWM_Stop(&htim1,ENB_Pin);
}
void turnMotor_R(int32_t Duty)
{
	if (Duty > 7200)
		Duty = 7200;
	if (Duty < -7200)
		Duty = -7200;

	if (Duty > 0)
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
	}
	else if (Duty < 0)
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	Duty = abs(Duty);
	htim1.Instance->CCR1 = Duty;
}
void turnMotor_L(int32_t Duty)
{
	if (Duty > 7200)
		Duty = 7200;
	if (Duty < -7200)
		Duty = -7200;

	if (Duty > 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
	}
	else if (Duty < 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
	}
	Duty = abs(Duty);
	htim1.Instance->CCR3 = Duty;
}
