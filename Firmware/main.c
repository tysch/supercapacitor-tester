#include <stdint.h>

#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

#include "xprintf.h"
#include "driver_5110_lcd.h"

#include "display.h"
#include "filters.h"
#include "adc.h"
#include "beep.h"
#include "led_blink.h"

#if 0
/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Settings
//
/////////////////////////////////////////////////////////////////////////////////////////////


#define NRH_PER_CPS 7000                // multiplier, nR/h per one pulse per second

#define BUTTON_LONG_PRESS_DELAY 2       // in seconds

#define MAX_VOLTAGE 4200                // Voltage range for battery
#define MIN_VOLTAGE 3000

#define SHUTDOWN_VOLTAGE 3200           // Undervoltage shutdown threshold

/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Global data
//
/////////////////////////////////////////////////////////////////////////////////////////////

static volatile uint32_t seconds = 0;

static volatile uint32_t counts = 0;
static volatile uint32_t prev_counts = 0;

static volatile uint32_t nrh = 0;
static volatile uint8_t  error = 0;

static volatile uint32_t background_nrh = 0;
static volatile uint8_t background_err = 0;

static volatile uint8_t backlight = 0;
static volatile uint8_t is_beeping;
static volatile uint8_t batt_charge;

uint32_t invsqrt(uint32_t n);


static volatile uint32_t abs_time_us = 0;
static volatile uint32_t abs_time_s = 0;

static volatile uint32_t rise_time_us = 0;
static volatile uint32_t rise_time_s = 0;

static volatile uint32_t fall_time_us = 0;
static volatile uint32_t fall_time_s = 0;


/*
        static volatile uint64_t abs_time_us = 0;
        static volatile uint64_t rise_time_us = 0;  
        static volatile uint64_t fall_time_us = 0;
*/

int new_pulse_came = 0;


/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Initizlizations
//
/////////////////////////////////////////////////////////////////////////////////////////////


void watchdog_timer_init (void) // Fires each 3,5 seconds
{
  /* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);   //Enable LSI Oscillator
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {}// Wait till LSI is ready
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);         // Enable Watchdog
	IWDG_SetPrescaler(IWDG_Prescaler_32);                 // 4, 8, 16 ... 256
	IWDG_SetReload(0x0FFF);                     //This parameter must be a number between 0 and 0x0FFF.
	IWDG_ReloadCounter();                       //Initial watchdog reset
	IWDG_Enable();
}

void SetHSE(void)
{
    ErrorStatus HSEStartUpStatus;
    // SYSCLK, HCLK, PCLK2 and PCLK1 configuration 
    RCC_DeInit();
    // Enable HSE 
    RCC_HSEConfig(RCC_HSE_ON);
    // Wait till HSE is ready 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if (HSEStartUpStatus == SUCCESS)
    {
        // HCLK = SYSCLK
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
        // PCLK2 = HCLK
        RCC_PCLK2Config( RCC_HCLK_Div1);
        // PCLK1 = HCLK/2
        RCC_PCLK1Config( RCC_HCLK_Div2);
        // PLLCLK = 8MHz * 9 = 72 MHz 
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
        // Enable PLL 
        RCC_PLLCmd(ENABLE);
        // Wait till PLL is ready 
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        // Select PLL as system clock source 
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
        /// Wait till PLL is used as system clock source 
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { // If HSE fails to start-up, the application will have wrong clock configuration.
        while (1)
        {
        }
    }
}

// Fires once per second and updates readings
void init_display_timer(void) 
{
    // Enable TIM4
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200;
    //TIMER_InitStructure.TIM_Prescaler = 800;
    TIMER_InitStructure.TIM_Period = 10000;
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
 
    // Enable the TIM4_IRQn Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// Fires once per second and updates readings
void init_10_ms_timer(void) 
{
    // Enable TIM3
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 72;
    //TIMER_InitStructure.TIM_Prescaler = 800;
    TIMER_InitStructure.TIM_Period = 10000;
    TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
 
    // Enable the TIM4_IRQn Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// Interrupts upon pulse from geiger counter on pin PB1
void geiger_counter_input_init()
{
    // Set variables used
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // Enable clock for AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Set pin as input
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    // Add IRQ vector to NVIC
    // PB1 is connected to EXTI_Line1, which has EXTI1_IRQn vector
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    // Set maximum possible priority 
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    // Set sub priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    // Enable interrupt
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // Add to NVIC
    NVIC_Init(&NVIC_InitStruct);

    // Tell system that you will use PB1 for EXTI_Line1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    // PD0 is connected to EXTI_Line1
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    // Enable interrupt
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // Interrupt mode
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // Triggers on rising and falling edge
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    // Add to EXTI
    EXTI_Init(&EXTI_InitStruct);
}




/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Main program flow
//
/////////////////////////////////////////////////////////////////////////////////////////////




// Called upon each particle detection
void EXTI1_IRQHandler(void)
{
    // Make sure that interrupt flag is set
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) 
    {
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) != 0)
        {
            if(!new_pulse_came)
            {
                        //rise_time_us = abs_time_us + TIM_GetCounter(TIM3);
                rise_time_us = abs_time_us + TIM_GetCounter(TIM3);
                rise_time_s = abs_time_s;

                while(rise_time_us > 1000000)
                {
                    rise_time_us -= 1000000;
                    rise_time_s++;
                }

                new_pulse_came = 1;
            }

            // Total particles detected
         //   if(is_beeping) beep();
          // led_blink();
        }
        // Clear interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}


void compute_and_display_values(void)
{


    // Compute charge and discharge time
    static int32_t prev_rise_time_us = 0;
    static uint32_t prev_rise_time_s = 0;

    static int32_t prev_fall_time_us = 0;
    static uint32_t prev_fall_time_s = 0;

    int32_t rel_rise_time_us = rise_time_us - prev_rise_time_us;
    uint32_t rel_rise_time_s = rise_time_s - prev_rise_time_s;

    int32_t rel_fall_time_us = fall_time_us - prev_fall_time_us;
    uint32_t rel_fall_time_s = fall_time_s - prev_fall_time_s;

    if(rel_rise_time_us < 0)
    {
        rel_rise_time_us += 1000000;
        rel_rise_time_s--;
    }

    if(rel_fall_time_us < 0)
    {
        rel_fall_time_us += 1000000;
        rel_fall_time_s--;
    }

    prev_rise_time_us = rise_time_us;
    prev_rise_time_s = rise_time_s;

    prev_fall_time_us = fall_time_us;
    prev_fall_time_s = fall_time_s;


                //static uint64_t prev_rise_time_us = 0;
//static uint64_t prev_fall_time_us = 0;

                //uint64_t rel_rise_time_us = rise_time_us - prev_rise_time_us;
//uint64_t rel_fall_time_us = fall_time_us - prev_fall_time_us;

            //prev_rise_time_us = rise_time_us;

            // prev_fall_time_us = fall_time_us;


    


/*

    static int seconds_since_disp = 0;
    int time_s;
    int time_us;

    seconds_since_disp++;

    if(counts > max_ticks) // Enough ticks to calculate t = 1/f
    {
        counts /= seconds_since_disp;
        time_s = 0;
        time_us = 1000000/counts;
        counts = 0;
        seconds_since_disp = 0;
    }
    else if((seconds_since_disp > max_ticks) && counts )
    {
        time_s = seconds_since_disp / counts;
        time_us = (seconds_since_disp % counts);
        time_us *= 1000000;
        seconds_since_disp = 0;
        counts = 0;
    }
    else 
    {
        return;
    }
*/



   // uint32_t tmp;
   // uint32_t err;

   // uint32_t fast_nrh;
    //uint32_t fast_error;

    // Check for integer overflows
   // if(counts > (2*1000*1000*1000/NRH_PER_CPS)) reset_counts();

   // tmp = NRH_PER_CPS * counts / seconds; // compute averaged intensity

   // if(tmp < background_nrh) nrh = 0;
   // else nrh = tmp - background_nrh;      // subtract background if it is set

    //if(background_err == 0) err = invsqrt(counts); // compute error
  //  else
  //  {
    //    if(nrh == 0) err = 99;
  //      else err = (background_err * background_nrh + invsqrt(counts)*tmp) / (nrh); 
  //      if(err > 99) err = 99;
   // }

    //fast_nrh = median_filter(NRH_PER_CPS*(counts-prev_counts)); // median filtered 5 last readings
  //  fast_error = invsqrt(fast_nrh / NRH_PER_CPS);
    LCD5110_clear();
 //   update_display(is_beeping, batt_charge, nrh, err, fast_nrh, fast_error);


update_display(0, 0, rel_rise_time_s, 0, rel_rise_time_us, 0);
                //update_display(0, 0, rel_rise_time_us / 1000000, 0, rel_rise_time_us % 1000000, 0);
}

// Updates display reading once per second and measures battery charge
void TIM4_IRQHandler(void)
{
    //uint32_t voltage;
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        //++seconds;

        //voltage = vcc_voltage();
       // batt_charge = (voltage - MIN_VOLTAGE) / ((MAX_VOLTAGE - MIN_VOLTAGE) / 100);

        //button_1_shutdown(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
       // button_2_toggle_beeping(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2));

       if(new_pulse_came ) 
       {
           compute_and_display_values();
           new_pulse_came = 0;
        }


       // prev_counts = counts;

       // IWDG_ReloadCounter();

        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void TIM3_IRQHandler(void)
{
    //uint32_t voltage;
    //static int ticks_10ms = 0;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
       // ticks_10ms++;
       // if(ticks_10ms >= 100)
      //  {
                    abs_time_us += 10000;
                    if(abs_time_us > 1000000) 
                    {
                        abs_time_us = 0;
                        abs_time_s++;
                              if(new_pulse_came ) 
       {
           compute_and_display_values();
           new_pulse_came = 0;
        }
                    }

       //     ticks_10ms = 0;
      //  }
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}


int main(void)
{
   // pushbuttons_gpio_init();
  //  watchdog_timer_init();
  //  IWDG_ReloadCounter();

    //if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) // Waked up upon IWDT reset
    //    shutdown();
   
    SetHSE();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    LCD5110_init();
    LCD5110_Led(0); 
    LCD5110_set_XY(0,0);
    LCD5110_write_string("Initializing..");

    //pushbuttons_init();
    //vcc_voltage_monitor_init();
    //beep_init();
    //led_init();

    //init_display_timer();
    geiger_counter_input_init();
    init_10_ms_timer();

    // ENABLE Wake Up Pin
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    //PWR_WakeUpPinCmd(ENABLE);

    while(1)
    {
     //       LCD5110_clear();
    //update_display(1, 1, i++, 1, 1, 1);

     //   __WFE();
    }
}
#endif




#include <stdint.h>

#include "stm32_lib/stm32f10x.h"
#include "stm32_lib/stm32f10x_conf.h"
#include "stm32_lib/system_stm32f10x.h"

#include "xprintf.h"
#include "driver_5110_lcd.h"

#include "display.h"
#include "filters.h"
#include "adc.h"
#include "beep.h"
#include "led_blink.h"



/////////////////////////////////////////////////////////////////////////////////////////////
//
//    Global data
//
/////////////////////////////////////////////////////////////////////////////////////////////

static volatile int32_t abs_time_s = 0;
static volatile int32_t abs_time_us = 0;
static volatile int32_t abs_time_10ms = 0;

void SetHSE(void)
{
    ErrorStatus HSEStartUpStatus;
    // SYSCLK, HCLK, PCLK2 and PCLK1 configuration 
    RCC_DeInit();
    // Enable HSE 
    RCC_HSEConfig(RCC_HSE_ON);
    // Wait till HSE is ready 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if (HSEStartUpStatus == SUCCESS)
    {
        // HCLK = SYSCLK
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
        // PCLK2 = HCLK
        RCC_PCLK2Config( RCC_HCLK_Div1);
        // PCLK1 = HCLK/2
        RCC_PCLK1Config( RCC_HCLK_Div2);
        // PLLCLK = 8MHz * 9 = 72 MHz 
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
        // Enable PLL 
        RCC_PLLCmd(ENABLE);
        // Wait till PLL is ready 
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        // Select PLL as system clock source 
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
        /// Wait till PLL is used as system clock source 
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { // If HSE fails to start-up, the application will have wrong clock configuration.
        while (1)
        {
        }
    }
}

// Fires once per second and updates readings
void init_10ms_timer(void) 
{
    // Enable TIM3
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200;
    //TIMER_InitStructure.TIM_Prescaler = 800;
    TIMER_InitStructure.TIM_Period = 10000;
    TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
 
    // Enable the TIM4_IRQn Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// Interrupts upon pulse from geiger counter on pin PB1
void geiger_counter_input_init()
{
    // Set variables used
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // Enable clock for AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Set pin as input
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Add IRQ vector to NVIC
    // PB1 is connected to EXTI_Line1, which has EXTI1_IRQn vector
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    // Set maximum possible priority 
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    // Set sub priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    // Enable interrupt
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // Add to NVIC
    NVIC_Init(&NVIC_InitStruct);

    // Tell system that you will use PB1 for EXTI_Line1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

        // PD0 is connected to EXTI_Line0
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    // Enable interrupt
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // Interrupt mode
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // Triggers on rising and falling edge
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    // Add to EXTI
    EXTI_Init(&EXTI_InitStruct);
}


static volatile int32_t rise_time_s;
static volatile int32_t rise_time_us;
static volatile int32_t fall_time_s;
static volatile int32_t fall_time_us;


static volatile int32_t fall_time_10ms;
static volatile int32_t rise_time_10ms;

void compute_and_display_values(void)
{
    update_display(0, 0, rise_time_s, 0, rise_time_us, 0);
}

// Called upon each particle detection
void EXTI0_IRQHandler(void)
{
    static int toggle = 0;
    // Make sure that interrupt flag is set
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) 
    {
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) != 0)
        {
            if(abs_time_s > 0) // Time >> displaying delay
            {
                rise_time_us = 100*TIM_GetCounter(TIM3);
                rise_time_s = abs_time_s;
                compute_and_display_values();
                TIM_SetCounter(TIM3, 0);
                abs_time_s = 0;
            }
            else
            {
                toggle ^= 1;
                if(toggle)
                {
                    rise_time_us = 100*TIM_GetCounter(TIM3);
                    rise_time_s = abs_time_s;
                    compute_and_display_values();
                }
                else
                {
                    TIM_SetCounter(TIM3, 0);
                    abs_time_s = 0;
                }
            }
        }
        // Clear interrupt flag
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// Updates display reading once per second and measures battery charge

void TIM3_IRQHandler(void)
{
    //uint32_t voltage;
    //static int ticks_10ms = 0;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        abs_time_s++;
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

void mode_sw_init(void)
{
    GPIO_InitTypeDef GPIOB_Init;

    GPIOB_Init.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIOB_Init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOB_Init.GPIO_Mode = GPIO_Mode_Out_PP;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(GPIOB, &GPIOB_Init);

    GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET); 
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET); 
}

int main(void)
{

    SetHSE();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    LCD5110_init();
    LCD5110_Led(0); 
    LCD5110_set_XY(0,0);
    LCD5110_write_string("Initializing..");

    mode_sw_init();
    init_10ms_timer();
    geiger_counter_input_init();

    while(1)
    {
     //   update_display(0, 0, rise_time_us, 0, fall_time_us, 0);
    }
}
