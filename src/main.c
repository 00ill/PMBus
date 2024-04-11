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

unsigned char dataByteLow = 0;
unsigned char dataByteLow_Bin[8];
unsigned char dataByteHigh = 0;
unsigned char dataByteHigh_Bin[8];
unsigned char Command = 0x19;

float Temperature = 0;
float Vin = 0;
float Vout = 0;
float Iout = 0;
float TempData = 0;
float power = 0;

int p = 0;
int p_fre = 3000;
unsigned char ChildAddress = 0x27;

float check11 = 0;
unsigned char checkH = 0xE9;
unsigned char CheckL = 0x85;

int ack_check[3] = {0};
int d;
int WaitingTime = INT32_MAX;

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
void L112DEC(unsigned char HighByte, unsigned char LowByte, float *Save_p)
{
	unsigned char sign = ((HighByte & 0x80) == 0x80); //맨 처음 비트로 지수 부분 부호 결정
	unsigned char exponent = (HighByte >> 3); // HighByte의 뒷 3자리 제거
	exponent = ~exponent; //반전
	exponent = (exponent & 0x1F); //맨 뒤 5자리만 남기고 제거
	exponent = exponent + 1; // +1

	unsigned int mantissa = ((HighByte & 0x07) << 8); //highbyte의 뒷 3자리만 저장
	mantissa = mantissa | LowByte; // LowData와 합침

	float exponent_Result = 1;
	unsigned char i;
	// 부호에 따라 계산 2^N 계산 부분
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
			exponent_Result *= 0.5;
		}
	}
	*Save_p = exponent_Result * mantissa; // 2^N * Y
}

void UL162DEC(unsigned char HighByte, unsigned char LowByte, float *Save_p)
{
	unsigned int mantissa = HighByte;
	mantissa = mantissa <<8;
	mantissa = mantissa | LowByte;
	
	*Save_p = 0.001953125 * mantissa;
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

void StartI2C_YYS(void)
{
	SDAOutput;
	SDAHigh;
	SCLHigh;
	Delay();
	Delay();
	SDALow;
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

void WaitForAck(void)
{
	for (d = 0; d < WaitingTime; d++)
	{
		if (SDARead == 0)
		{
			WaitingTime = 0;
		}
	}
	WaitingTime = INT32_MAX;
}

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
		Delay();
		Delay();

		pre_byte = pre_byte << 1;
		SCLLow;
	}
	Delay();
	SDALow;
	SDAHigh;
	SDAInput;
	WaitForAck();
	SCLHigh;
	Delay();
	Delay();
	Delay();
	ack_check[checkNum] = SDARead;
	SCLLow;
	Delay();
	SDAOutput;
	SDAHigh;
	Delay();
}

unsigned char I2C_Read_SDC(unsigned char nack)
{
	unsigned char data, i, bit;
	unsigned char j, k;
	unsigned char z;
	SDAHigh;
	Delay();
	Delay();
	SCLLow;
	for (i = 0; i < 8; i++)
	{
		SCLHigh;
		Delay();
		Delay();
		Delay();
		for (k = 0; k < 10; k++)
		{
		}
		data = data << 1;
		bit = SDARead;
		data = data | bit;
		SDAOutput;
		SCLLow;
		Delay();
	}
	if (nack)
	{
		SDAHigh;
	}
	else
	{
		SDALow;
	}
	SCLHigh;
	Delay();
	Delay();
	Delay();
	Delay();
	SCLLow;
	Delay();
	return data;
}

void ReadWord(unsigned char command)
{
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4, 0);
	I2C_Write_SDC(command, 1, 1);
	Delay();
	Delay();
	StartI2C_YYS();
	I2C_Write_SDC((ChildAddress << 1) + 1, 3, 2);
	dataByteLow = I2C_Read_SDC(false);
	dataByteHigh = I2C_Read_SDC(true);
	Delay();
	StopI2C();
}
unsigned char BlockReadData[5] = {0};
void BlockRead(unsigned char command)
{
	unsigned char b = 0;
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4, 0);
	I2C_Write_SDC(command, 1, 1);
	Delay();
	Delay();
	StartI2C_YYS();
	I2C_Write_SDC((ChildAddress << 1) + 1, 3, 2);
	for(b = 0 ; b < 4; b++)
	{
		BlockReadData[b] = I2C_Read_SDC(false);
	}
	BlockReadData[4] = I2C_Read_SDC(true);
	Delay();
	StopI2C();
}
void ReadVin(void)
{	
	ReadWord(0x88);
	L112DEC(dataByteHigh, dataByteLow, &Vin);
	dataByteHigh = 0;
	dataByteLow = 0;
}

void ReadVout(void)
{
	ReadWord(0x8B);
	UL162DEC(dataByteHigh, dataByteLow, &Vout);
	dataByteHigh = 0;
	dataByteLow = 0;
}

void ReadIout(void)
{
	ReadWord(0x8C);
	L112DEC(dataByteHigh, dataByteLow, &Iout);
	//UL162DEC(dataByteHigh, dataByteLow, &Iout);
			Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
			Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
}

void ReadTemp(void)
{
	ReadWord(0x8D);
	//L112DEC(dataByteHigh, dataByteLow, &Temperature);
	UL162DEC(dataByteHigh, dataByteLow, &Temperature);
	dataByteHigh = 0;
	dataByteLow = 0;
}

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
		if (p > 3000)
		{
			//BlockRead(0x30);
			ReadWord(0x88);
			UL162DEC(dataByteHigh, dataByteLow, &Temperature);
			Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
			Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
			ReadVin();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			ReadVout();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			ReadIout();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			ReadTemp();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			Delay();
			power = Iout * Vout;
			p = 0;
		}
	}
}
