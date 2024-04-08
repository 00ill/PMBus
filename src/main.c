/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2018, NXP.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * Description:
 * =====================================================================
 * This short project is a starting point to learn GPIO.
 * An input is polled to detect a high or low level. An output is set
 * depending on input state. If running code on the S32K14x evaluation
 * board, pressing button 0 lights up the blue LED.
 */

#include "device_registers.h"
#include <S32K146.h>

#define EVB 0
#if (EVB)
#define PTA0 0
#define PTA1 1
#else
#define PTD9 9
#define PTD8 8
#endif

#define new_Ver 1

#if (EVB)
#define SCLInput SCLSetDirectionFunc(0)
#define SCLOutput SCLSetDirectionFunc(1)
#define SCLHigh PTA->PSOR |= 1 << PTA0
#define SCLLow PTA->PCOR |= 1 << PTA0
#define SCLRead (((unsigned int)(PTA->PDIR)) & (1 << 0))

#define SDAInput SDASetDirectionFunc(0)
#define SDAOutput SDASetDirectionFunc(1)
#define SDAHigh PTA->PSOR |= 1 << PTA1
#define SDALow PTA->PCOR |= 1 << PTA1
#define SDARead (((unsigned int)(PTA->PDIR) & (1 << 1))) >> 1

#define GPIO_PDDR_PDD_MASK 0xFFFFFFFFu
#define GPIO_PDDR_PDD_SHIFT 0u
#define GPIO_PDDR_PDD_WIDTH 32u
#define GPIO_PDDR_PDD(x) (((unsigned int)(((unsigned int)(x)) << GPIO_PDDR_PDD_SHIFT)) & GPIO_PDDR_PDD_MASK)
#else
#define SCLInput SCLSetDirectionFunc(0)
#define SCLOutput SCLSetDirectionFunc(1)
#define SCLHigh PTD->PSOR |= 1 << PTD9
#define SCLLow PTD->PCOR |= 1 << PTD9
#define SCLRead (((unsigned int)(PTD->PDIR)) & (1 << 0))

#define SDAInput SDASetDirectionFunc(0)
#define SDAOutput SDASetDirectionFunc(1)
#define SDAHigh PTD->PSOR |= 1 << PTD8
#define SDALow PTD->PCOR |= 1 << PTD8
#define SDARead (((unsigned int)(PTD->PDIR) & (1 << PTD8))) >> PTD8
#define READ_SDA(_pval) (GPIO_ReadPin(GPIOID_SDA, _pval))
#define GPIO_PDDR_PDD_MASK 0xFFFFFFFFu
#define GPIO_PDDR_PDD_SHIFT 0u
#define GPIO_PDDR_PDD_WIDTH 32u
#define GPIO_PDDR_PDD(x) (((unsigned int)(((unsigned int)(x)) << GPIO_PDDR_PDD_SHIFT)) & GPIO_PDDR_PDD_MASK)
#endif

void WDOG_disable(void)
{
	WDOG->CNT = 0xD928C520;	  /* Unlock watchdog 		*/
	WDOG->TOVAL = 0x0000FFFF; /* Maximum timeout value 	*/
	WDOG->CS = 0x00002100;	  /* Disable watchdog 		*/
}

void Dec2Bin_8bit(unsigned char dec_value, unsigned char *binArrp)
{
	unsigned char temp_dec = dec_value;
	unsigned char *temp_Arrp = binArrp;
	unsigned char i;
	for (i = 0; i < 8; i++)
	{
		(*temp_Arrp) = ((temp_dec & 0x80) == 0x80);
		temp_Arrp++;
		temp_dec = temp_dec << 1;
	}
}

void L112DEC(unsigned char HighByte, unsigned char LowByte, unsigned char *Save_p)
{
	unsigned char sign = ((HighByte & 0x10) == 0x10);
	unsigned char exponent = (HighByte >> 3);
	exponent = ~exponent;
	exponent = exponent & 0x1F;
	exponent = exponent + 1;

	unsigned int mantissa = ((HighByte & 0x07) << 8);
	mantissa = mantissa | LowByte;

	float exponent_Result = 1;
	unsigned char i;
	if (sign == 0)
	{
		for (i = 0; i < exponent; i++)
		{
			exponent_Result *= 2;
		}
	}
	else
	{
		for (i = 0; i < exponent; i++)
		{
			exponent_Result /= 2;
		}
	}
	*Save_p = exponent_Result * mantissa;
}

#if (EVB)
void SCLSetDirectionFunc(unsigned char dir)
{
	unsigned int direction = PTA->PDDR;
	direction &= (unsigned int)(~((unsigned int)1U << 0));
	direction |= (unsigned int)((unsigned int)dir << 0);
	PTA->PDDR = GPIO_PDDR_PDD(direction);
}

void SDASetDirectionFunc(unsigned char dir)
{
	unsigned int direction = PTA->PDDR;
	direction &= (unsigned int)(~((unsigned int)1U << 1));
	direction |= (unsigned int)((unsigned int)dir << 1);
	PTA->PDDR = GPIO_PDDR_PDD(direction);
}
#else
void SCLSetDirectionFunc(unsigned char dir)
{
	unsigned int direction = PTD->PDDR;
	direction &= (unsigned int)(~((unsigned int)1U << 9));
	direction |= (unsigned int)((unsigned int)dir << 9);
	PTD->PDDR = GPIO_PDDR_PDD(direction);
}

void SDASetDirectionFunc(unsigned char dir)
{
	unsigned int direction = PTD->PDDR;
	direction &= (unsigned int)(~((unsigned int)1U << 8));
	direction |= (unsigned int)((unsigned int)dir << 8);
	PTD->PDDR = GPIO_PDDR_PDD(direction);
}
#endif

void Delay(void)
{
	int i;
	for (i = 0; i < 20; i++)
	{
	}
}
int nack_fg = 0;
#if (new_Ver)
void StartI2C_YYS(void)
{
	SDAOutput;
	SDAHigh;
	SCLHigh;
	Delay();
	Delay();
	SDALow;
	// Delay();
}

void StopI2C(void)
{
	SDALow;
	Delay();
	SCLHigh;
	Delay();
	SDAHigh;
}

void StopI2C_YYS(void)
{
	SDAOutput;
	SCLHigh;
	Delay();
	SDALow;
	Delay();
	SDALow;
	Delay();
	SDAHigh;
	Delay();
}
int ack_check[3] = {0};
void I2C_Write_SDC(unsigned char pre_byte, unsigned char delay, unsigned char checkNum)
{
	int i, temp;
	int j;
	SDAOutput;
	SCLLow;
	for (j = 0; j < delay; j++)
	{
		Delay();
	}
	// Delay();
	// Delay();
	for (i = 0; i < 8; i++)
	{
		temp = pre_byte & 0x80;
		if (temp)
		{
			SDAHigh;
		}
		else
		{
			SDALow;
		}
		Delay();
		SCLHigh;
		Delay();

		pre_byte = pre_byte << 1;
		SCLLow;
	}
	Delay();
	SDALow;
	 SDAHigh;
	 SDAInput;
	SCLHigh;
	Delay();
	ack_check[checkNum] = SDARead;
	SCLLow;
	Delay();
	SDAHigh;
	Delay();
}

unsigned char I2C_Read_SDC(unsigned char nack, unsigned char delay)
{
	unsigned char data, i, bit;
	unsigned char j, k;
	unsigned char z;
	SDAHigh;
	Delay();
	// SDALow;
	//  for(z = 0; z<5; z++)
	//  {

	// }
	Delay();
	SCLLow;
	// Delay();
	for (j = 0; j < delay; j++)
	{
		Delay();
	}
	// Delay();
	// SDALow;
	for (i = 0; i < 8; i++)
	{
		SCLHigh;
		// Delay();
		for (k = 0; k < 10; k++)
		{
		}
		// SDALow;
		SDAHigh;
		SDAInput;
		data = data << 1;
		bit = SDARead;
		data = data | bit;
		SDAOutput;
		SCLLow;
		Delay();
	}
	SCLHigh;
	if (nack)
	{
		SDAHigh;
	}
	else
	{
		SDALow;
	}
	Delay();
	SCLLow;
	Delay();
	if (nack_fg == 1)
	{

		if (nack)
		{
			SCLHigh;
		}
		else
		{
			SDAHigh;
		}
		Delay();
	}

	return data;
}
#else if
void StartI2C_YYS(void)
{
	SDAOutput;
	SDAHigh;
	SCLHigh;
	Delay();
	SDALow;
	Delay();
}

void StopI2C(void)
{
	SDALow;
	Delay();
	SCLHigh;
	Delay();
	SDAHigh;
}

void I2C_Write_SDC(unsigned char pre_byte, unsigned char trash)
{
	int i, temp;

	SDAOutput;
	SCLLow;
	Delay();
	Delay();
	Delay();

	for (i = 0; i < 8; i++)
	{
		temp = pre_byte & 0x80;
		if (temp)
		{
			SDAHigh;
		}
		else
		{
			SDALow;
		}
		SCLHigh;
		Delay();

		pre_byte = pre_byte << 1;
		SCLLow;
		Delay();
	}

	SDALow;
	SCLHigh;
	Delay();
	SCLLow;
	Delay();
	SDAHigh;
	Delay();
}

unsigned char I2C_Read_SDC(unsigned char nack, unsigned char trash)
{
	unsigned char data, i, bit;

	SDAHigh;
	Delay();
	Delay();
	Delay();
	SCLLow;
	Delay();
	Delay();

	for (i = 0; i < 8; i++)
	{
		SCLHigh;
		Delay();
		SDAInput;
		data = data << 1;
		bit = SDARead;
		data = data | bit;
		SDAOutput;
		SCLLow;
		Delay();
	}
	SCLHigh;
	if (nack)
	{
		SDAHigh;
	}
	else
	{
		SDALow;
	}
	Delay();
	SCLLow;
	Delay();
	// if(nack)
	// {
	//     SCLHigh;
	// }
	// else
	// {
	//     SDAHigh;
	// }
	// Delay();
	return data;
}

#endif
int test = 0;
int test_old = 0;
unsigned char dataByteLow = 0;
unsigned char dataByteLow_Bin[8];
unsigned char dataByteHigh = 0;
unsigned char dataByteHigh_Bin[8];
unsigned char Command = 0x88;

unsigned int Temperature = 0;
unsigned int Vin = 0;
unsigned int Vout = 0;
unsigned int Iout = 0;
unsigned int TempData = 0;

int p = 0;
int p_fre = 100000;
unsigned char ChildAddress = 0x27;
int bit_7_fg = 0;

unsigned char SendHigh = 0;
unsigned char SendLow = 0;
int main(void)
{
	WDOG_disable();
	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;

#if (EVB)
	/* Configure port D0 as GPIO output (LED on EVB) */
	PTA->PDDR |= 1 << PTA0;			 /* Port D0: Data Direction= output */
	PTA->PDDR |= 1 << PTA1;			 /* Port D0: Data Direction= output */
	PORTA->PCR[0] = PORT_PCR_MUX(1); /* Port D0: MUX = GPIO */
	PORTA->PCR[1] = PORT_PCR_MUX(1); /* Port D0: MUX = GPIO */
#else
	PTD->PDDR |= 1 << PTD8;			 /* Port D0: Data Direction= output */
	PTD->PDDR |= 1 << PTD9;			 /* Port D0: Data Direction= output */
	PORTD->PCR[8] = PORT_PCR_MUX(1); /* Port D0: MUX = GPIO */
	PORTD->PCR[9] = PORT_PCR_MUX(1); /* Port D0: MUX = GPIO */
#endif

	SDAOutput;
	SCLOutput;
	SDAHigh;
	SCLHigh;

	for (;;)
	{
		if (p > -1)
		{
			p++;
		}

		if (p >= p_fre)
		{
			if (bit_7_fg == 1)
			{
				StartI2C_YYS();
				//// I2C_Write_SDC(0xAE, 4);
				I2C_Write_SDC(ChildAddress, 4,0);
				I2C_Write_SDC(Command, 1,1);
				Delay();
				Delay();
				StartI2C_YYS();
				I2C_Write_SDC(ChildAddress + 1, 3,2);
				//// I2C_Write_SDC(0xAF, 3);
				// dataByteLow = I2C_Read_SDC(false, 0);
				dataByteHigh = I2C_Read_SDC(true, 0);
				Delay();
				StopI2C();
				Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
				Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
			}
			else if (bit_7_fg == 0)
			{
				StartI2C_YYS();
				//// I2C_Write_SDC(0xAE, 4);
				I2C_Write_SDC(ChildAddress << 1, 4,0);
				I2C_Write_SDC(Command, 1,1);
				Delay();
				Delay();
				// I2C_Write_SDC(SendHigh, 3);
				// I2C_Write_SDC(SendLow, 3);
				StartI2C_YYS();
				I2C_Write_SDC((ChildAddress << 1) + 1, 3,2);
				//// I2C_Write_SDC(0xAF, 3);
				//dataByteLow = I2C_Read_SDC(false, 0);
				dataByteHigh = I2C_Read_SDC(true, 0);

				Delay();
				StopI2C();
				Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
				Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
			}
			else
			{
				// StartI2C_YYS();
				// I2C_Write_SDC(0xAE, 4);
				// /// I2C_Write_SDC(0x40, 4);
				// I2C_Write_SDC(Command, 1);
				// Delay();
				// Delay();
				// StartI2C_YYS();
				// /// I2C_Write_SDC(0x41, 3);
				// I2C_Write_SDC(0xAF, 3);
				// // dataByteLow = I2C_Read_SDC(false, 0);
				// dataByteHigh = I2C_Read_SDC(true, 0);
				// Delay();
				// StopI2C();
				// Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
				// Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
				// L112DEC(dataByteHigh, dataByteLow, &TempData);
			}
			p = 0;
		}

		if (test != test_old)
		{
			StartI2C_YYS();
			I2C_Write_SDC(0x22, 4,0);
			/// I2C_Write_SDC(0x40, 4);
			I2C_Write_SDC(Command, 1,1);
			Delay();
			Delay();
			StartI2C_YYS();
			/// I2C_Write_SDC(0x41, 3);
			I2C_Write_SDC(0x23, 3,2);
			// dataByteLow = I2C_Read_SDC(false, 0);
			dataByteHigh = I2C_Read_SDC(true, 0);
			Delay();
			StopI2C();
			Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
			Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
			L112DEC(dataByteHigh, dataByteLow, &TempData);
			test_old = test;
		}
	}
}