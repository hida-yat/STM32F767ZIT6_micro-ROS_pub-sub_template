/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "usart.h"

#include <std_msgs/msg/int32.h> //publisherで使用するデータ型
#include <geometry_msgs/msg/twist.h> //subscriberで使用するデータ型

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//エラーハンドリング用のマクロ
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//micro-ROS関連の変数の初期化。ここにグローバル変数として宣言　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　自分が宣言したい変数を書く必要あり
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//publisherの
std_msgs__msg__Int32 encoder_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void LED_blink(const void *subscribed_msg);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
//エンコーダスタート

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

  // micro-ROSの設定。
  //　※　ここは基本いじらなくてよいはず
  rmw_uros_set_custom_transport(
    true,
	(void *) &huart3,
	cubemx_transport_open,
	cubemx_transport_close,
	cubemx_transport_write,
	cubemx_transport_read
  );

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // ここからmicro-ROSのセットアップ
  allocator = rcl_get_default_allocator(); // デフォルトのメモリアロケータを取得
  node = rcl_get_zero_initialized_node(); // 初期化された空のノードを取得
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //初期化設定の作成

  //　※　ここまで基本触らなくて良いはず

  //ノードの作成。publisherでもsubscriberでも必要
  RCCHECK(rclc_node_init_default(&node, "f7_node", "", &support));

  //publisherの作成
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/f767zi_encoder"));


  //subscriverの作成
  RCCHECK(rclc_subscription_init_default(
	&subscriber,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	  "cmd_vel"));

  //エンコーダのカウントを0で初期化しておく
  encoder_msg.data = 0;

  // コールバックを管理ためのexecutor
  // Subscriber、Timer、Serviceなどもコールバック関数を設定する
  // Publisherだけなら、以降の処理は必要ない
  int callback_size = 1;	// コールバックを行う数
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, callback_size, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, //第３引数のcmd_vel_msgは受信したデータを格納する変数。とりあえず設定しといたらいい。
								   &LED_blink, ON_NEW_DATA); // LED_blinkはコールバック関数（後ほど記載）

  /* Infinite loop */
  for(;;)
  {
	//pulishするデータ(encoder_msg.data)にエンコーダの計測値を格納する。
	encoder_msg.data = TIM2->CNT;
	//publisherがencoder_msgをpublishする
	RCSOFTCHECK(rcl_publish(&publisher, &encoder_msg, NULL));

	//subscriptionを実行する
	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)););

	//100msのDelayを入れる
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* subscriberのコールバック関数を宣言。　引数はsubscribeしたメッセージを表すが、
   その型はconst void*型にしておく。ただし、関数内で適切な型にキャストする必要がある*/
void LED_blink(const void *subscribed_msg){
  //subscribed_msgは、subscriberが受け取ったメッセージのポインタ。geometry_msgs__msg__Twistにキャストしてcmd_vel_msgに格納。
  //cmd_vel_msgについて、rclc_executor_add_subscription()の第３引数と同じであるが、その必要は特にない
  const geometry_msgs__msg__Twist *cmd_vel_msg = (const geometry_msgs__msg__Twist *)subscribed_msg;

  //受け取ったcmd_velに対してlinear.x(並進x方向の速度)が0以上であれば
  if(cmd_vel_msg->linear.x > 0){
    //LD1点灯
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
  }else{
    //LD1消灯
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
  }
}

/* USER CODE END Application */

