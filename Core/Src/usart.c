/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
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

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 RX uses circular DMA so FOC execution cannot drop UART bytes. */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_DMA_DeInit(uartHandle->hdmarx);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "motor/foc.h"
#include "motor/motor_runtime_param.h"
#include "global_def.h"

#define MOTOR_COMMAND_BUFFER_SIZE 128
#define MOTOR_UART_DMA_BUFFER_SIZE 256

static char motor_uart_rx_buffer[MOTOR_COMMAND_BUFFER_SIZE];
static uint8_t motor_uart_dma_buffer[MOTOR_UART_DMA_BUFFER_SIZE];
static volatile uint16_t motor_uart_rx_index;
static volatile uint8_t motor_uart_command_ready;
static volatile uint32_t motor_uart_last_rx_tick;
static volatile uint16_t motor_uart_dma_last_pos;

static const char *control_type_name(motor_control_type type);
static void motor_uart_store_rx_byte(uint8_t byte);
static void motor_uart_dma_process(void);

static void motor_uart_log_event(const char *event)
{
  printf("{\"type\":\"event\",\"event\":\"%s\",\"tick\":%lu,\"mode\":\"%s\",\"iq_ref_a\":%.3f,\"iq_a\":%.3f,\"id_ref_a\":%.3f,\"id_a\":%.3f,\"iq_ref_norm\":%.3f,\"iq_norm\":%.3f,\"id_ref_norm\":%.3f,\"id_norm\":%.3f}\r\n",
         event,
         (unsigned long)HAL_GetTick(),
         control_type_name(motor_control_context.type),
         motor_target_i_q,
         motor_i_q,
         motor_target_i_d,
         motor_i_d,
         motor_target_i_q / MAX_CURRENT,
         motor_i_q / MAX_CURRENT,
         motor_target_i_d / MAX_CURRENT,
         motor_i_d / MAX_CURRENT);
}

void motor_uart_event_heartbeat(void)
{
  motor_uart_log_event("heartbeat");
}

int fputc(int c, FILE *stream)
{
  // 发送保持阻塞式；接收由 DMA 独立搬运，不会依赖 UART 中断抢占 FOC。
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET)
    ;
  huart2.Instance->DR = (uint8_t)c;
  return c;
}

static float command_constrain(float value, float min_value, float max_value)
{
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

static const char *control_type_name(motor_control_type type)
{
  switch (type)
  {
  case control_type_position:
    return "position";
  case control_type_speed:
    return "speed";
  case control_type_torque:
    return "torque";
  case control_type_speed_torque:
    return "speed_torque";
  case control_type_position_speed_torque:
    return "position_speed_torque";
  default:
    return "null";
  }
}

static void print_command_help(void)
{
  printf("# pid current <p> <i> <d>\r\n");
  printf("# pid id|iq|speed|position <p> <i> <d>\r\n");
  printf("# target id|iq <norm>, target speed <rad/s>, target position <deg>\r\n");
  printf("# limit torque <norm>, limit speed <rad/s>\r\n");
  printf("# mode null|torque|speed|speed_torque|position|position_speed_torque\r\n");
  printf("# show, help\r\n");
}

static void print_motor_config(void)
{
  printf("# mode=%s position=%.3fdeg speed=%.3frad/s id=%.3f iq=%.3f max_speed=%.3f max_torque=%.3f\r\n",
         control_type_name(motor_control_context.type),
         rad2deg(motor_control_context.position),
         motor_control_context.speed,
         motor_control_context.torque_norm_d,
         motor_control_context.torque_norm_q,
         motor_control_context.max_speed,
         motor_control_context.max_torque_norm);
  printf("# pid position=%.6f,%.6f,%.6f speed=%.6f,%.6f,%.6f\r\n",
         motor_pid_position.p, motor_pid_position.i, motor_pid_position.d,
         motor_pid_speed.p, motor_pid_speed.i, motor_pid_speed.d);
  printf("# pid id=%.6f,%.6f,%.6f iq=%.6f,%.6f,%.6f\r\n",
         motor_pid_torque_d.p, motor_pid_torque_d.i, motor_pid_torque_d.d,
         motor_pid_torque_q.p, motor_pid_torque_q.i, motor_pid_torque_q.d);
}

static void execute_motor_command(char *command)
{
  char group[20] = {0};
  char name[32] = {0};
  float value_1 = 0;
  float value_2 = 0;
  float value_3 = 0;

  if (strcmp(command, "help") == 0)
  {
    print_command_help();
    return;
  }
  if (strcmp(command, "show") == 0)
  {
    print_motor_config();
    return;
  }

  if (sscanf(command, "%19s %31s %f %f %f", group, name,
             &value_1, &value_2, &value_3) == 5 &&
      strcmp(group, "pid") == 0)
  {
    if (motor_control_context.type != control_type_null)
    {
      motor_uart_log_event("pid_rejected_busy");
      printf("# ERR pid busy: use mode null before changing PID\r\n");
      return;
    }
    if (strcmp(name, "current") == 0)
    {
      set_torque_d_pid(value_1, value_2, value_3);
      set_torque_q_pid(value_1, value_2, value_3);
    }
    else if (strcmp(name, "id") == 0)
      set_torque_d_pid(value_1, value_2, value_3);
    else if (strcmp(name, "iq") == 0)
      set_torque_q_pid(value_1, value_2, value_3);
    else if (strcmp(name, "speed") == 0)
      set_speed_pid(value_1, value_2, value_3);
    else if (strcmp(name, "position") == 0)
      set_position_pid(value_1, value_2, value_3);
    else
    {
      printf("# ERR unknown pid: %s\r\n", name);
      return;
    }
    printf("# OK pid %s %.6f %.6f %.6f\r\n",
           name, value_1, value_2, value_3);
    motor_uart_log_event("pid_applied");
    return;
  }

  if (sscanf(command, "%19s %31s %f", group, name, &value_1) == 3)
  {
    if (strcmp(group, "target") == 0)
    {
      if (strcmp(name, "iq") == 0)
        motor_control_context.torque_norm_q = command_constrain(value_1, -1.0f, 1.0f);
      else if (strcmp(name, "id") == 0)
        motor_control_context.torque_norm_d = command_constrain(value_1, -1.0f, 1.0f);
      else if (strcmp(name, "speed") == 0)
        motor_control_context.speed = value_1;
      else if (strcmp(name, "position") == 0)
        motor_control_context.position = deg2rad(value_1);
      else
      {
        printf("# ERR unknown target: %s\r\n", name);
        return;
      }
      printf("# OK target %s %.6f\r\n", name, value_1);
      motor_uart_log_event("target_applied");
      return;
    }

    if (strcmp(group, "limit") == 0)
    {
      if (strcmp(name, "torque") == 0)
        motor_control_context.max_torque_norm = command_constrain(value_1, 0.0f, 1.0f);
      else if (strcmp(name, "speed") == 0)
        motor_control_context.max_speed = fabsf(value_1);
      else
      {
        printf("# ERR unknown limit: %s\r\n", name);
        return;
      }
      printf("# OK limit %s %.6f\r\n", name, value_1);
      motor_uart_log_event("limit_applied");
      return;
    }
  }

  if (sscanf(command, "%19s %31s", group, name) == 2 &&
      strcmp(group, "mode") == 0)
  {
    motor_control_type new_type = control_type_null;
    if (strcmp(name, "torque") == 0)
      new_type = control_type_torque;
    else if (strcmp(name, "speed") == 0)
      new_type = control_type_speed;
    else if (strcmp(name, "speed_torque") == 0)
      new_type = control_type_speed_torque;
    else if (strcmp(name, "position") == 0)
      new_type = control_type_position;
    else if (strcmp(name, "position_speed_torque") == 0)
      new_type = control_type_position_speed_torque;
    else if (strcmp(name, "null") != 0)
    {
      printf("# ERR unknown mode: %s\r\n", name);
      return;
    }

    motor_control_context.type = new_type;
    if (new_type == control_type_null)
    {
      set_pwm_duty(0.0f, 0.0f, 0.0f);
      motor_control_context.torque_norm_d = 0.0f;
      motor_control_context.torque_norm_q = 0.0f;
      motor_control_context.speed = 0.0f;
      reset_motor_pid_states();
      motor_uart_log_event("motor_stopped");
    }
    printf("# OK mode %s\r\n", control_type_name(new_type));
    if (new_type != control_type_null)
      motor_uart_log_event("mode_changed");
    return;
  }

  motor_uart_log_event("command_error");
  printf("# ERR command: %s\r\n", command);
}

void motor_uart_command_start(void)
{
  motor_uart_rx_index = 0;
  motor_uart_command_ready = 0;
  motor_uart_last_rx_tick = HAL_GetTick();
  motor_uart_dma_last_pos = 0;

  // 接收改为循环 DMA，避免 FOC 高优先级执行期间 UART RXNE 溢出丢字节。
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  if (HAL_UART_Receive_DMA(&huart2, motor_uart_dma_buffer, MOTOR_UART_DMA_BUFFER_SIZE) != HAL_OK)
  {
    motor_uart_log_event("uart_dma_error");
    printf("# UART RX DMA ERROR\r\n");
    return;
  }
  motor_uart_log_event("uart_dma_ready");
  printf("# UART RX DMA READY\r\n");
}

void motor_uart_command_process(void)
{
  char command[MOTOR_COMMAND_BUFFER_SIZE];
  uint32_t primask;

  motor_uart_dma_process();

  // 兼容没有附加CR/LF的串口工具：最后一个字节后静默100ms即视为一条完整命令
  if (!motor_uart_command_ready && motor_uart_rx_index > 0 &&
      HAL_GetTick() - motor_uart_last_rx_tick >= 100)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    if (!motor_uart_command_ready && motor_uart_rx_index > 0)
    {
      motor_uart_rx_buffer[motor_uart_rx_index] = '\0';
      motor_uart_command_ready = 1;
      motor_uart_rx_index = 0;
    }
    if (!primask)
      __enable_irq();
  }

  if (!motor_uart_command_ready)
    return;

  primask = __get_PRIMASK();
  __disable_irq();
  strncpy(command, motor_uart_rx_buffer, sizeof(command));
  command[sizeof(command) - 1] = '\0';
  motor_uart_command_ready = 0;
  if (!primask)
    __enable_irq();

  execute_motor_command(command);
}

static void motor_uart_store_rx_byte(uint8_t byte)
{
  motor_uart_last_rx_tick = HAL_GetTick();
  if (motor_uart_command_ready)
    return;

  if (byte == '\r' || byte == '\n')
  {
    if (motor_uart_rx_index > 0)
    {
      motor_uart_rx_buffer[motor_uart_rx_index] = '\0';
      motor_uart_command_ready = 1;
      motor_uart_rx_index = 0;
    }
    return;
  }

  if (motor_uart_rx_index < MOTOR_COMMAND_BUFFER_SIZE - 1)
    motor_uart_rx_buffer[motor_uart_rx_index++] = (char)byte;
  else
    motor_uart_rx_index = 0;
}

static void motor_uart_dma_process(void)
{
  uint16_t dma_pos = MOTOR_UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

  while (motor_uart_dma_last_pos != dma_pos)
  {
    motor_uart_store_rx_byte(motor_uart_dma_buffer[motor_uart_dma_last_pos]);
    motor_uart_dma_last_pos++;
    if (motor_uart_dma_last_pos >= MOTOR_UART_DMA_BUFFER_SIZE)
      motor_uart_dma_last_pos = 0;
  }
}

void motor_uart_command_irq_handler(void)
{
  // 保留接口以兼容 stm32f1xx_it.c；USART2 RX 已由 DMA 接管。
}
/* USER CODE END 1 */
