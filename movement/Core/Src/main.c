/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "string.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
   #define BIT0   0x01 // 0000 000'0' BIT0 for wall N,
   #define BIT1   0x02 // 0000 00'0'0 BIT1 for wall E
   #define BIT2   0x04 // 0000 0'0'00 BIT2 for wall S
   #define BIT3   0x08 // 0000 '0'000 BIT3 for wall W
   #define BIT4   0x10 // 000'0' 0000
   #define BIT5   0x20 // 00'0'0 0000
   #define BIT6   0x40 // 0'0'00 0000
   #define BIT7   0x80 /* '0'000 0000 for checking if car ran over or not */

#define CHECKBIT(x,b) (x&b) // check bit "b" in byte "x"
#define INPUT_A1 GPIO_PIN_5 // Output port for motor 1
#define INPUT_A2 GPIO_PIN_6 // Output port for motor 1
#define INPUT_B3 GPIO_PIN_0 // Output port for motor 2
#define INPUT_B4 GPIO_PIN_1 // Output port for motor 2
#define TRIG_PIN GPIO_PIN_12
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_13
#define ECHO_PORT GPIOB
#define TRIG_PIN_1 GPIO_PIN_8
#define TRIG_PORT_1 GPIOA
#define ECHO_PIN_1 GPIO_PIN_9
#define ECHO_PORT_1 GPIOA
#define TRIG_PIN_2 GPIO_PIN_14
#define TRIG_PORT_2 GPIOB
#define ECHO_PIN_2 GPIO_PIN_15
#define ECHO_PORT_2 GPIOB
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint32_t Value3 = 0;
uint32_t Value4 = 0;
uint32_t Value5 = 0;
uint32_t Value6 = 0;
uint16_t Distance  = 0;
int stkptr=0;
int stk_empty_flag = 0;
float Kp = 0.25, Ki = 0, Kd = 0.8;
int error = 0;
float P = 0, I = 0, D = 0, PID_value = 0;
int speed_right=80;// ENA duty cycle
int speed_left=80;// ENB duty cycle
float previous_error = 0;
uint16_t Distance_1=0;// Distance_left
uint16_t Distance_2=0;// Distance_right
unsigned char mazeflood[36]={
4,3,2,2,3,4,
3,2,1,1,2,3,
2,1,0,0,1,2,
2,1,0,0,1,2,
3,2,1,1,2,3,
4,3,2,2,3,4
};

uint16_t current_cell=30;
unsigned char walldata[36];
unsigned char current_dir=1; // direction is forward at time 0s // west
uint16_t led_right=0,led_front=0,led_left=0;
uint16_t next_direct;
uint16_t destination = 15;
uint16_t count=0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/*
         7 6 5 4 3 2 1 0
direction: |0|W|0|S|0|E|0|N|
           64  16  4   1

*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void turn_right(){
	// phai_tien
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	//trai_dung
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	TIM2->CCR1=100;


	HAL_Delay(200);
}
void turn_left(){
	// phai_dung
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	//trai_tien
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	TIM2->CCR1=100;

	HAL_Delay(200);

}
void ford_ward(){
	// phai_tien
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	//trai_tien
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	TIM2->CCR1=40;
	HAL_Delay(200);

}
void stop(){
	// phai_tien
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	//trai_dung
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_Delay(1000);

}
void sensor_right(){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		      __HAL_TIM_SET_COUNTER(&htim1, 0);
		      while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		      HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

		      pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		      // wait for the echo pin to go high
		      while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
		      Value1 = __HAL_TIM_GET_COUNTER (&htim1);

		      pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		      // wait for the echo pin to go low
		      while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
		      Value2 = __HAL_TIM_GET_COUNTER (&htim1);

		      Distance = (Value2-Value1)* 0.034/2;
		      if ( Distance <= 12 ) led_right=1;
		      else led_right=0;

}
void sensor_left(){
	HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
		  	    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  	    HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin low

		  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  	    // wait for the echo pin to go high
		  	    while (!(HAL_GPIO_ReadPin (ECHO_PORT_1, ECHO_PIN_1)) && pMillis + 10 >  HAL_GetTick());
		  	    Value3 = __HAL_TIM_GET_COUNTER (&htim1);

		  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  	    // wait for the echo pin to go low
		  	    while ((HAL_GPIO_ReadPin (ECHO_PORT_1, ECHO_PIN_1)) && pMillis + 50 > HAL_GetTick());
		  	    Value4 = __HAL_TIM_GET_COUNTER (&htim1);

		  	    Distance_1 = (Value4-Value3)/2/29.412;
		  	    if (Distance_1 <= 12) led_left=1;
		  	    else led_left=0;
}
void sensor_forward(){
	 HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  	  	  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
		  	  	  	    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  	  	  	    HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);  // pull the TRIG pin low

		  	  	  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  	  	  	    // wait for the echo pin to go high
		  	  	  	    while (!(HAL_GPIO_ReadPin (ECHO_PORT_2, ECHO_PIN_2)) && pMillis + 10 >  HAL_GetTick());
		  	  	  	    Value5 = __HAL_TIM_GET_COUNTER (&htim1);

		  	  	  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  	  	  	    // wait for the echo pin to go low
		  	  	  	    while ((HAL_GPIO_ReadPin (ECHO_PORT_2, ECHO_PIN_2)) && pMillis + 50 > HAL_GetTick());
		  	  	  	    Value6 = __HAL_TIM_GET_COUNTER (&htim1);

		  	  	  	   Distance_2 = (Value6-Value5)/2/29.412;
		  	  	  	   if (Distance_2 <= 12) led_front=1;
		  	  	  	   else led_front=0;

}

void flood_fill()
{
  // ford_ward();
  // stop();
   unsigned char neighbour_val[] = {255,255,255,255};
   unsigned char wallinfo = 0, x=0;
   int stk[1000]; // Stack of 1000 cell, stk_ptr pointer, stk_empty_flag = 0 when stack is not empty, = 1 when empty
   uint16_t floodcell = 0;
   stk[0] = 255;
   stkptr++;
   stk[stkptr] = current_cell;
   while (stk_empty_flag == 0)
   {
	   //ford_ward();
	   //stop();

      floodcell = stk[stkptr];

      if (stkptr == 1) stk_empty_flag = 1;
      else
      stkptr--;

      neighbour_val[0] = 255;
      neighbour_val[1] = 255;
      neighbour_val[2] = 255;
      neighbour_val[3] = 255;

      wallinfo = walldata[floodcell];
      // flood_cell = current cell, maze_flood like a new array for checking wall
      // If there is no wall, the cell at that direction is called neighbor_cell with current_cell
      if ((wallinfo & BIT0) == 0 )  // Check BIT data of NORTH wall, if not wall -> save the value into maze_flood
         neighbour_val[0] = mazeflood[floodcell - 10]; //

      //East
      if ((wallinfo & BIT1)==0 )   // same
         neighbour_val[1] = mazeflood[floodcell + 1];

      //South
      if ((wallinfo & BIT2)==0 )  // same
         neighbour_val[2] = mazeflood[floodcell + 10];

      //West
      if ((wallinfo & BIT3 )==0)   // same
         neighbour_val[3] = mazeflood[floodcell - 1];

       // after save value of neighbor_cell, we will find the smallest value of 4
      // we will save the smallest in to neighbor_val[0]
      x = 0;
      while (x < 3)
      {
    	   //ford_ward();
    	   //stop();
         if (neighbour_val[0] > (neighbour_val[x + 1]))
            neighbour_val[0] = neighbour_val[x + 1];//

         x+=1;
      }
   // if current_cell != smallest neighbor_cell, we will update the value of current cell = neighbor_cell+ 1
      if ((floodcell != destination) && (mazeflood[floodcell] != (1+neighbour_val[0])))

      {
    	  mazeflood[floodcell] = 1 + neighbour_val[0];         // update

         //North
         if ((wallinfo & BIT0) ==0) // Due to update value, we need to update all, so take the value of NORTH cell into stack for check
         {
            stkptr++;
            stk[stkptr] = floodcell - 10;
            stk_empty_flag = 0;
         }


         //East
         if ((wallinfo & BIT1)==0)// same
         {
            stkptr++;
            stk[stkptr] = floodcell + 1;
            stk_empty_flag = 0;
         }


         //South
         if ((wallinfo & BIT2)== 0)// same
         {
            stkptr++;
            stk[stkptr] = floodcell + 10;
            stk_empty_flag = 0;
         }


         //West
         if ((wallinfo & BIT3)==0) // same
         {
            stkptr++;
            stk[stkptr] = floodcell - 1;
            stk_empty_flag = 0;
         }
  	  // ford_ward();
  	  // stop();
      }
 	  // ford_ward();
 	  // stop();
   }
	  // ford_ward();
	  // stop();
}   //End flood_fill

uint16_t stepper(void)
{
	 //  ford_ward();
	  // stop();
   uint16_t neighbour_val[4] = {255,255,255,255};
   uint16_t min=1;
   unsigned char wallinfo = 0, x = 0, x2 = 0;

   wallinfo = walldata[current_cell];


// after have a new value, we save the new into neighbor_val for finding the smallest and turn the direction to it
   if ((wallinfo & BIT0)== 0)
      neighbour_val[0] = mazeflood[current_cell - 6];


   if ((wallinfo & BIT1) == 0)
      neighbour_val[1] = mazeflood[current_cell + 1];


   if ((wallinfo & BIT2)==0)
      neighbour_val[2] = mazeflood[current_cell + 6];


   if ((wallinfo & BIT3)==0)
      neighbour_val[3] = mazeflood[current_cell - 1];

   // find smallest
   x = 0;
   x2 = 0;
   while (x < 3)
   {

      if (neighbour_val[0] > neighbour_val[x + 1])
      {
         x2 = x + 1;
         min = pow(2, ( 2 * x2 )) ; // first, neighbor_val [4] have the value of 1 ( direction of North)
         // if the value of neighbor_val from 1 to 3 is smaller, we will update the direction of neighbor_val[4]to 4, 16, or 64 ( E -> S -> W)
         neighbour_val[0] = neighbour_val[x + 1];

      }
      x += 1;
   }
  // consider the direction of car now with neighbor_val[4]
   if (current_dir == min) // same direction, move straight
   {
      min = 1; // straight
      return min;
   }
   // if direction == 64 and neighbor_val[4] == 1 -> direction follow North, from W to N, turn right
   if ( ((current_dir == 64) && (min == 1)) || (current_dir == (min/4)) )
   {
      min = 2; // right
      return min;
   }
   // the same reason
   if ( ((current_dir == 1) && (min == 64)) || (current_dir == (4 * min)) )
   {
      min = 3; //left
      return min;
   }
   else
   {
	  min =4; //turn around
	  return min;
   }

}

 /*void get_sensor(TRIG_PORT,TRIG_PIN,ECHO_PORT,ECHO_PIN,Distance){


}*/
void caculate_PID(){
P=error;
I=+error;
D=error-previous_error;
PID_value=Kp*P+Ki*I+Kd*D;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    walldata[0]=9;
    walldata[5]=3;
    walldata[30]=12;
    walldata[35]=6;
    for (int i=1;i<5;i++)
    {
      walldata[i]=1;
    }
    for (int i=6;i<25;i+=6)
    {
      walldata[i]=8;
    }
    for (int i=11;i<30;i+=10)
    {
      walldata[i]=2;
    }
    for (int i=31;i<35;i++)
    {
      walldata[i]=4;
    }
	walldata[current_cell]=((walldata[current_cell]) | (1<<7) | (1<<1) | (0<<0) );

	//HAL_Delay(500);
    /*ford_ward();
    if (current_dir==1)current_cell=current_cell+10; //NORTH
    if (current_dir==4)current_cell=current_cell+1;  // EAST
    if (current_dir==16)current_cell=current_cell-10; // SOUTH
    if (current_dir==64)current_cell=current_cell-1; //WEST
    */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  int timer_1=HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//PA1 TIm2 channel 2 ENB
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);//PA11 TIM1 channel 4 ENA

 // HAL_GPIO_WritePin(GPIOB, TRIG_PIN_1, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOB, TRIG_PIN_2, GPIO_PIN_RESET);
  HAL_TIM_Base_Start(&htim1);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (count==0)
	  {
		  HAL_Delay(5000);
		  ford_ward();
		  stop();
		   if (current_dir==1)current_cell=current_cell-6; //NORTH
		   if (current_dir==4)current_cell=current_cell+1;  // EAST
		   if (current_dir==16)current_cell=current_cell+6; // SOUTH
		   if (current_dir==64)current_cell=current_cell-1; //WEST
			walldata[current_cell]=((walldata[current_cell]) | (1<<7) | (0<<1) | (1<<0) );

		   count++;
	  }
	  else
	  {
		  sensor_right();
		  sensor_left();
		  sensor_forward();
	      if (current_dir == 1 ) // North direction
	      {
	    	  if ((walldata [current_cell] & BIT7) == 0)
	    		  walldata[current_cell]=((walldata[current_cell]) | (1<<7) | (led_left<<3) | (led_right<<1) | (led_front<<0) | (0<<2) );

			  if ( ((walldata[current_cell+1] & BIT7)==0) && (current_cell != 5) && (current_cell != 11)
					  && (current_cell != 17) && (current_cell != 23) && (current_cell != 29) && (current_cell != 35)
					  && led_right==1 )
          {
				  walldata[current_cell+1]=((walldata[current_cell+1]) | (1<<7) | (1<<3));
          }



			  if (((walldata[current_cell-1] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 6)
					  && (current_cell != 12) && (current_cell != 18) && (current_cell != 24) && (current_cell != 30)
					  && led_left==1 )
        {

          walldata[current_cell-1]=((walldata[current_cell-1]) | (1<<7) | (1<<1));

        }

			  if (((walldata[current_cell-6] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 1)
					  && (current_cell != 2) && (current_cell != 3) && (current_cell != 4) && (current_cell != 5)
					  && led_front==1 )
        {
          walldata[current_cell-6]=((walldata[current_cell-6]) | (1<<7) | (1<<2));

        }
	      }

	      if (current_dir ==4 ) //East direction
	      {
	    	  if ((walldata [current_cell]& BIT7) == 0)
	    		  walldata[current_cell]=((walldata[current_cell]) | (1<<7) | (led_left<<0) | (led_right<<2) | (led_front<<1) | (0<<3)) ;
			  if (((walldata[current_cell+1] & BIT7)==0 ) && (current_cell != 5) && (current_cell != 11)
					  && (current_cell != 17) && (current_cell != 23) && (current_cell != 29) && (current_cell != 35)
					  && led_front==1 )
				  walldata[current_cell+1]=((walldata[current_cell+1]) | (1<<7) | (1<<3));

			  if (((walldata[current_cell-6] & BIT7)==0 ) && (current_cell != 30) && (current_cell != 31)
					  && (current_cell != 32) && (current_cell != 33) && (current_cell != 34) && (current_cell != 35)
					  && led_left==1 )
				  walldata[current_cell-6]=((walldata[current_cell-6]) | (1<<7) | (1<<2));

			  if (((walldata[current_cell+6] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 1)
					  && (current_cell != 2) && (current_cell != 3) && (current_cell != 4) && (current_cell != 5)
					  && led_right==1 )
				  walldata[current_cell+6]=((walldata[current_cell+6]) | (1<<7) | (1<<0));
	      }

	      if (current_dir ==16 )// South direction
	      {

	    	  if ((walldata [current_cell]& BIT7) == 0)
	    		  walldata[current_cell]=((walldata[current_cell]) | (1<<7) | (led_left<<1) | (led_right<<3) | (led_front<<2) | (0<<0) );
			  if (((walldata[current_cell+1] & BIT7)==0 ) && (current_cell != 5) && (current_cell != 11)
					  && (current_cell != 17) && (current_cell != 23) && (current_cell != 29) && (current_cell != 35)
					  && led_left==1 )
				  walldata[current_cell+1]=((walldata[current_cell+1]) | (1<<7) | (1<<3));

			  if (((walldata[current_cell-1] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 6)
					  && (current_cell != 12) && (current_cell != 18) && (current_cell != 24) && (current_cell != 30)
					  && led_right==1 )
				  walldata[current_cell-1]=((walldata[current_cell-1]) | (1<<7) | (1<<1));

			  if (((walldata[current_cell+6] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 1)
					  && (current_cell != 2) && (current_cell != 3) && (current_cell != 4) && (current_cell != 5)
					  && led_front==1 )
				  walldata[current_cell+6]=((walldata[current_cell+6]) | (1<<7) | (1<<2));
	      }

	      if (current_dir ==64 )// West direction
	      {

	    	  if ((walldata [current_cell]& BIT7) == 0)
	    		  walldata[current_cell]=((walldata[current_cell]) | (1<<7) | (led_left<<2) | (led_right<<0) | (led_front<<3) | (0<<1) );
			  if (((walldata[current_cell-1] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 6)
					  && (current_cell != 12) && (current_cell != 18) && (current_cell != 24) && (current_cell != 30)
						  && led_right==1 )
					  walldata[current_cell-1]=((walldata[current_cell-1]) | (1<<7) | (1<<1));

			  if (((walldata[current_cell-6] & BIT7)==0 ) && (current_cell != 30) && (current_cell != 31)
					  && (current_cell != 32) && (current_cell != 33) && (current_cell != 34) && (current_cell != 35)
					  && led_left==1 )
				  walldata[current_cell-6]=((walldata[current_cell-6]) | (1<<7) | (1<<2));

			  if (((walldata[current_cell+6] & BIT7)==0 ) && (current_cell != 0) && (current_cell != 1)
					  && (current_cell != 2) && (current_cell != 3) && (current_cell != 4) && (current_cell != 5)
					  && led_front==1 )
				  walldata[current_cell+6]=((walldata[current_cell+6]) | (1<<7) | (1<<2));

	      }
      flood_fill();
	   //ford_ward();
	   //stop();
      next_direct=stepper();
      if (next_direct == 2)
      {
    	  turn_right();
    	  current_dir=current_dir*4;
    	  if (current_dir ==0) 	current_dir =1;
    	  ford_ward();
      }
      else if(next_direct == 3)
      {
    	  turn_left();
    	  current_dir=current_dir/4;
    	  if (current_dir == 0)
    		  current_dir =1;
    	  ford_ward();
      }
      else if (next_direct == 4)// turn back 180 degrees
      {
    	  turn_right(); // turn 1 time
    	  current_dir= current_dir *4;
    	  if (current_dir ==0) 	current_dir =1;

    	  turn_right(); // turn 2 times
    	  current_dir= current_dir *4;
    	  if (current_dir ==0) 	current_dir =1;
    	  ford_ward();
      }
      else
      ford_ward();
      stop();
      if (current_dir==1)current_cell=current_cell-6; //NORTH
      if (current_dir==4)current_cell=current_cell+1;  // EAST
      if (current_dir==16)current_cell=current_cell+6; // SOUTH
      if (current_dir==64)current_cell=current_cell-1; //WEST

      if (current_cell == destination) // when the car go to final -> make the car go out and pick up
      {
    	  if (current_dir == 1)
    		  ford_ward();
    	  else if (current_dir == 4)
    	  {
    		  turn_left();
    		  current_dir /= 4;
        	  if (current_dir == 0)
        		  current_dir =1;
    		  ford_ward();
    	  }
    	  else if (current_dir == 64)
    	  {
    		  turn_right();
    		  current_dir *= 4;
        	  if (current_dir == 0)
        		  current_dir =1;
    		  ford_ward();
    	  }
    	  break;
      }
    HAL_Delay(3000);

    //toggle_pin();

	 // get_sensor(GPIOA, TRIG_PIN_1, GPIOA, ECHO_PIN_1, Distance_1);

     //HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, 50);
	 // control();

//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, speed_right);// set ENB
//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, speed_left);//set ENA

  }
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
