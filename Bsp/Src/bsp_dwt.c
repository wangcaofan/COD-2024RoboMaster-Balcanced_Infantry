/**
  ******************************************************************************
  * @file           : bsp_dwt.c
  * @brief          : bsp DWT functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_dwt.h"
#include "stm32f4xx.h"

DWT_Time_Typedef SysTime;
uint64_t CYCCNT64;
/* Private variables ---------------------------------------------------------*/
static uint32_t CPU_FREQ_Hz;
static uint32_t CPU_FREQ_Hz_ms;
static uint32_t CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Update the DWT Count.
  */
static void DWT_CNT_Update(void);


/**
  * @brief Initializes the DWT according to writing the specified data 
  *        to the internal configuration registers.
  * @param CPU_Freq_mHz: MCU clock frequence
  * @retval none
  */
void BSP_DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* Initializes the DWT peripheral */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* clear the DWT CYCCNT register */
    DWT->CYCCNT = (uint32_t)0u;

    /* Enable Cortex-M DWT CYCCNT register */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* update the clock frequence */
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}
//------------------------------------------------------------------------------

/**
  * @brief get the DWT timedelta.
  * @param none
  * @retval DWT timedelta
  */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}
//------------------------------------------------------------------------------

/**
  * @brief get the DWT timedelta.
  * @param none
  * @retval DWT timedelta
  */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}
//------------------------------------------------------------------------------

/**
  * @brief Update the DWT timeline.
  * @param none
  * @retval none
  */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}
//------------------------------------------------------------------------------

/**
  * @brief get the DWT second timeline.
  * @param none
  * @retval none
  */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}
//------------------------------------------------------------------------------

/**
  * @brief get the DWT millisecond timeline.
  * @param none
  * @retval none
  */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}
//------------------------------------------------------------------------------

/**
  * @brief get the DWT microsecond timeline.
  * @param none
  * @retval none
  */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}
//------------------------------------------------------------------------------

/**
  * @brief Update the DWT Count.
  * @param none
  * @retval none
  */
static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;

    CYCCNT_LAST = cnt_now;
}
//------------------------------------------------------------------------------

/**
  * @brief DWT Delay.
  * @param Delay: delay microsecond
  * @retval none
  */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz);
}
//------------------------------------------------------------------------------
