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

#define OPERATION 0x01
#define CLEAR_FAULTS 0x03
#define STORE_USER_CODE 0x17
#define CAPABILITY 0x19
#define VOUT_MODE 0x20
#define VOUT_COMMAND 0x21
#define VOUT_TRANSITION_RATE 0x27
#define IOUT_OC_FAULT_LIMIT 0x46
#define STATUS_BYTE 0x78
#define STATUS_WORD 0x79
#define READ_VIN 0x88
#define READ_VOUT 0x8B
#define READ_IOUT 0x8C
#define READ_TEMPERATURE 0x8D
#define MFR_ID 0x99
#define MFR_VOUT_MIN 0xA4
#define MFR_VOUT_MAX 0xA5
#define MFR_STATUS_FAULTS 0xF0

typedef enum FAULT
{
	FAULT_NOTHING,
	TEMPERATURE_FAULTS_OR_WARNING,
	PMBUS_COMMUNICATION_EVENT
} fault;

unsigned char dataByteLow = 0;
unsigned char dataByteLow_Bin[8];
unsigned char dataByteHigh = 0;
unsigned char dataByteHigh_Bin[8];
unsigned char singleByteData = 0;
unsigned char faultDataHigh = 0;
unsigned char faultDataLow = 0;
fault FAULT_STATE = FAULT_NOTHING;

float SettingV = 14;
float SettingV_old = 14;
unsigned char transhigh = 0;
unsigned char translow = 0;

float Temperature = 0;
float Vin = 0;
float Vout = 0;
float Iout = 0;
float power = 0;

int p = 0;
int p_fre = 3000;
unsigned char ChildAddress = 0x27;

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
	if (LowByte != 0xFF)
	{
		unsigned char sign = ((HighByte & 0x80) == 0x80);
		unsigned char exponent = (HighByte >> 3);
		exponent = ~exponent;
		exponent = (exponent & 0x1F);
		exponent = exponent + 1;

		unsigned int mantissa = ((HighByte & 0x07) << 8);
		mantissa = mantissa | LowByte;
		float exponent_Result = 1;
		unsigned char i;

		if (((HighByte >> 3) & 0x1F) != 0x00)
		{
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
		}
		if ((exponent_Result * mantissa) < 900)
		{
			*Save_p = exponent_Result * mantissa;
		}
	}
}

void UL162DEC(unsigned char HighByte, unsigned char LowByte, float *Save_p)
{
		unsigned int mantissa = HighByte;
		mantissa = mantissa << 8;
		mantissa = mantissa | LowByte;

		*Save_p = 0.001953125 * mantissa;
}

void DEC2UL16(float Dec, unsigned char *HighByte_p, unsigned char *LowByte_p)
{
	unsigned int temp_Dec = (unsigned int)(Dec / 0.001953125);
	*HighByte_p = ((temp_Dec & 0xFF00) >> 8);
	*LowByte_p = (temp_Dec & 0xFF);
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
	for (i = 0; i < 30; i++)
	{
	}
}

void TermI2C(void)
{
	unsigned int i;
	for (i = 0; i < 100; i++)
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

void WaitForAck(void)
{
	int d;
	for (d = 0; d < INT32_MAX; d++)
	{
		if (SDARead == 0)
		{
			d = INT32_MAX;
		}
	}
}

void I2C_Write_SDC(unsigned char pre_byte, unsigned char delay)
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
	// SDALow;
	SDAHigh;
	SDAInput;
	WaitForAck();
	SCLHigh;
	Delay();
	Delay();
	Delay();
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
	Delay();
	SCLHigh;
	Delay();
	Delay();
	Delay();
	Delay();
	SCLLow;
	Delay();
	return data;
}

void ReadByte(unsigned char Command)
{
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4);
	I2C_Write_SDC(Command, 1);
	Delay();
	Delay();
	StartI2C_YYS();
	I2C_Write_SDC((ChildAddress << 1) + 1, 3);
	singleByteData = I2C_Read_SDC(true);
	Delay();
	StopI2C();
	TermI2C();
}

void WriteByte(unsigned char Command, unsigned char Data)
{
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4);
	I2C_Write_SDC(Command, 1);
	Delay();
	Delay();
	I2C_Write_SDC(Data, 1);
	Delay();
	StopI2C();
	TermI2C();
}

void ReadWord(unsigned char Command)
{
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4);
	I2C_Write_SDC(Command, 1);
	Delay();
	Delay();
	StartI2C_YYS();
	I2C_Write_SDC((ChildAddress << 1) + 1, 3);
	dataByteLow = I2C_Read_SDC(false);
	dataByteHigh = I2C_Read_SDC(true);
	Delay();
	StopI2C();
	TermI2C();
}

void WriteWord(unsigned char command, unsigned char DataByteLow, unsigned DataByteHigh)
{
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4);
	I2C_Write_SDC(command, 1);
	Delay();
	Delay();
	I2C_Write_SDC(DataByteLow, 1);
	I2C_Write_SDC(DataByteHigh, 1);
	Delay();
	StopI2C();
	TermI2C();
}

unsigned char BlockReadData[5] = {0};

void BlockRead(unsigned char command)
{
	unsigned char b = 0;
	StartI2C_YYS();
	I2C_Write_SDC(ChildAddress << 1, 4);
	I2C_Write_SDC(command, 1);
	Delay();
	Delay();
	StartI2C_YYS();
	I2C_Write_SDC((ChildAddress << 1) + 1, 3);
	for (b = 0; b < 4; b++)
	{
		BlockReadData[b] = I2C_Read_SDC(false);
	}
	BlockReadData[4] = I2C_Read_SDC(true);
	Delay();
	StopI2C();
}

void ReadVin(void)
{
	ReadWord(READ_VIN);
	L112DEC(dataByteHigh, dataByteLow, &Vin);
	TermI2C();
}

void ReadVout(void)
{
	ReadWord(READ_VOUT);
	UL162DEC(dataByteHigh, dataByteLow, &Vout);
	TermI2C();
}

void ReadIout(void)
{
	ReadWord(READ_IOUT);
	L112DEC(dataByteHigh, dataByteLow, &Iout);
	Dec2Bin_8bit(dataByteLow, &dataByteLow_Bin);
	Dec2Bin_8bit(dataByteHigh, &dataByteHigh_Bin);
	TermI2C();
}

void ReadTemp(void)
{
	ReadWord(READ_TEMPERATURE);
	L112DEC(dataByteHigh, dataByteLow, &Temperature);
	TermI2C();
}

void SetVout(float DesiredV)
{
	unsigned char dataByteHigh;
	unsigned char dataByteLow;
	DEC2UL16(DesiredV, &dataByteHigh, &dataByteLow);
	WriteWord(VOUT_COMMAND, dataByteLow, dataByteHigh);
}

void FaultCheck(void)
{
	ReadWord(STATUS_WORD);
	faultDataHigh = dataByteHigh;
	faultDataLow = dataByteLow;
	if (faultDataLow != 0xFF)
	{
		if ((faultDataLow & 0x04) == 0x04)
		{
			FAULT_STATE = TEMPERATURE_FAULTS_OR_WARNING;
		}
		else if ((faultDataLow & 0x02) == 0x02)
		{
			FAULT_STATE = PMBUS_COMMUNICATION_EVENT;
		}
		else
		{
			FAULT_STATE = FAULT_NOTHING;			
		}
	}
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
		if (p > p_fre)
		{
			TermI2C();
			ReadVin();
			ReadVout();
			ReadIout();
			ReadTemp();
			FaultCheck();

			power = Iout * Vout;

			if(SettingV != SettingV_old)
			{
				SetVout(SettingV);
				SettingV_old = SettingV;
			}
			p = 0;
		}
	}
}
