#include "bsp_systick.h"

static u16 fac_us = 0;
static u16 fac_ms = 0;

// Initialize Systick
void Systick_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us = SystemCoreClock / 8000000; // SystemCoreClock / 8000000 for 1us (assuming HCLK=72MHz, Div8=9MHz)
	fac_ms = (u16)fac_us * 1000;
}

// Delay microseconds
void Delay_us(uint32_t nus)
{
	u32 temp;
	SysTick->LOAD = nus * fac_us;			  // Load time value
	SysTick->VAL = 0x00;					  // Clear current value
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable Systick
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16))); // Wait for time to reach
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // Disable Systick
	SysTick->VAL = 0X00;					   // Clear current value
}

// Delay milliseconds
void Delay_ms(uint16_t nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms * fac_ms;		  // Load time value (SysTick->LOAD is 24bit)
	SysTick->VAL = 0x00;					  // Clear current value
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable Systick
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16))); // Wait for time to reach
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // Disable Systick
	SysTick->VAL = 0X00;					   // Clear current value
}
