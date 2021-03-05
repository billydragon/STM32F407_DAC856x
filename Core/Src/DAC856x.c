/*
 * DAC8563.c
 *
 *  Created on: Mar 5, 2021
 *      Author: billy
 */
#include <DAC856x.h>
#include "main.h"
#include "gpio.h"
#include "spi.h"


/**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    #define SPI1_CS_Pin 		GPIO_PIN_4
	#define SPI1_CS_GPIO_Port 	GPIOA
    */


#define CS_CLK_ENABLE() 			__HAL_RCC_GPIOF_CLK_ENABLE()
#define CS_GPIO						SPI1_CS1_GPIO_Port
#define CS_PIN						SPI1_CS1_Pin
#define CS_Disable()				CS_GPIO->BSRR = CS_PIN
#define CS_Enable()					CS_GPIO->BSRR = ((uint32_t)CS_PIN << 16U)

#define CS_DELAY_us					1

#define SPI_BUFFER_SIZE				4

#define DAC856X_INIT_SPI()			DAC856x_InitSPIParam(SPI_BAUDRATEPRESCALER_2, SPI_PHASE_1EDGE, SPI_POLARITY_LOW)

uint32_t g_spiLen;
uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];

#define X1	0
#define Y1  -100000

#define X2	65535
#define Y2  100000

void DAC856x_Init24()
{		DWT_Delay_Init();
		DAC856x_WriteCmd24((4 << 19) | (0 << 16) | (3 << 0));

		DAC856x_WriteCmd24((6 << 19) | (0 << 16) | (3 << 0));
		/* Enable if DAC8562 */
		//DAC856x_Set_Data(0, 32767);
		//DAC856x_Set_Data(1, 32767);

		DAC856x_WriteCmd24((7 << 19) | (0 << 16) | (1 << 0));
}

void DAC856x_Init()
{
	DWT_Delay_Init();
	DAC856x_WriteCmd(CMD_RESET_REG, DATA_RESET_ALL_REG);      			// reset
	DAC856x_WriteCmd(CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);        			// power up
	DAC856x_WriteCmd(CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN);      	// enable internal reference
	DAC856x_WriteCmd(CMD_GAIN, DATA_GAIN_B2_A2);            			// set multiplier
	DAC856x_WriteCmd(CMD_LDAC_DIS, DATA_LDAC_DIS_AB);          			// update the caches
}


void DAC856x_Set_CS(uint8_t _Level)
{
	if (_Level == CS_ENABLE)
	{
		DAC856X_INIT_SPI();
		CS_Enable();
		DWT_Delay_us(CS_DELAY_us);
	}
	else
	{
		DWT_Delay_us(CS_DELAY_us);
		CS_Disable();
	}
}

void DAC856x_Set_Data24(uint8_t _ch, uint16_t _dac)
{
		if (_ch == DAC_CH1)
		{
			/* Write to DAC-A input register and update DAC-A; */
			DAC856x_WriteCmd24((3 << 19) | (0 << 16) | (_dac << 0));
		}
		else if (_ch == DAC_CH2)
		{
			/* Write to DAC-B input register and update DAC-A; */
			DAC856x_WriteCmd24((3 << 19) | (1 << 16) | (_dac << 0));
		}

}

void DAC856x_Set_Data(uint8_t _ch, uint16_t _dac)
{
		if (_ch == DAC_CH1)
		{
			/* Write to DAC-A input register and update DAC-A; */
			DAC856x_WriteCmd(CMD_SET_A_UPDATE_A,_dac);
		}
		else if (_ch == DAC_CH2)
		{
			/* Write to DAC-B input register and update DAC-A; */
			DAC856x_WriteCmd(CMD_SET_B_UPDATE_B,_dac);
		}

}


void DAC856x_WriteCmd24(uint32_t _cmd)
{
	DAC856x_Set_CS(CS_ENABLE);
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (_cmd >> 16);
	g_spiTxBuf[g_spiLen++] = (_cmd >> 8);
	g_spiTxBuf[g_spiLen++] = (_cmd);
	DAC856x_spiTransfer();
	DAC856x_Set_CS(CS_DISABLE);

}

void DAC856x_WriteCmd(uint8_t _cmd, uint16_t _data)
{
	DAC856x_Set_CS(CS_ENABLE);
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (_cmd);
	g_spiTxBuf[g_spiLen++] = (_data >> 8) & 0xFF;
	g_spiTxBuf[g_spiLen++] = (_data) & 0xFF;
	DAC856x_spiTransfer();
	DAC856x_Set_CS(CS_DISABLE);

}


void DAC856x_spiTransfer(void)
{
	if (g_spiLen > SPI_BUFFER_SIZE)
	{
		return;
	}
	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)g_spiTxBuf, g_spiLen, 1000000) != HAL_OK)
	{
		Error_Handler();
	}
}

int32_t CaculTwoPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x)
{
return y1 + ((int64_t)(y2 - y1) * (x - x1)) / (x2 - x1);
}

int32_t DAC856x_DacToVoltage(uint16_t _dac)
{
	int32_t y;

	/* CaculTwoPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);*/
	y =  CaculTwoPoint(DAC_MIN, Y1, DAC_MAX, Y2, _dac);
	return y;
}


uint32_t DAC856x_VoltageToDac(int32_t _volt)
{
	/* CaculTwoPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);*/
	return CaculTwoPoint(Y1, DAC_MIN, Y2, DAC_MAX, _volt);
}


/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: clock cycle counter not started
 *         0: clock cycle counter works
 */
uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}

