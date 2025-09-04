/*
 * bmp280.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Admin
 */

#include "main.h"
#include "bmp280.h"

uint8_t Read8(BMP280_t *bmp, uint8_t Reggister)
{
	uint8_t Value;
	HAL_I2C_Mem_Read(bmp->i2c, ((bmp->Address)<<1), Reggister, 1, &Value, 1, BMP280_I2C_TIMEOUT);
	return Value;
}


uint16_t Read16(BMP280_t *bmp, uint8_t Reggister)
{
	uint8_t Value[2];
	HAL_I2C_Mem_Read(bmp->i2c, ((bmp->Address)<<1), Reggister, 1, Value, 2, BMP280_I2C_TIMEOUT);
	return ((Value[1] << 8) | Value[0]);
}

uint32_t Read24(BMP280_t *bmp, uint8_t Reggister)
{
	uint8_t Value[3];
	HAL_I2C_Mem_Read(bmp->i2c, ((bmp->Address)<<1), Reggister, 1, Value, 3, BMP280_I2C_TIMEOUT);
	return ((Value[0] << 16) | (Value[1] << 8) | Value[2]);
}

uint8_t Write8(BMP280_t *bmp, uint8_t Reggister, uint8_t Value)
{
	HAL_I2C_Mem_Write(bmp->i2c, ((bmp->Address)<<1), Reggister, 1, &Value, 1, BMP280_I2C_TIMEOUT);
	return 0;
}
//
//
//
//
//

void BMP280_SetMode(BMP280_t *bmp, uint8_t Mode)
{
	uint8_t Tmp;

	if(Mode > 3 ) Mode = 3;

	Tmp = Read8(bmp, BMP280_CONTROL);

	Tmp = Tmp & 0xFC; // Tmp (xxxx xx00) Mask(1111 1100)
	Tmp |= Mode & 0x03; //// Mask (0000 0011)

	Write8(bmp, BMP280_CONTROL, Tmp);
}

void BMP280_SetPressureOversampling(BMP280_t *bmp, uint8_t POversampling)
{
	uint8_t Tmp;

	if(POversampling > 5 ) POversampling = 5;

	Tmp = Read8(bmp, BMP280_CONTROL);

	Tmp = Tmp & 0xE3; // Tmp (xxx0 00xx) Mask (1110 0011)
	Tmp |= (POversampling << 2) & 0x1C; // Mask (0001 1100)

	Write8(bmp, BMP280_CONTROL, Tmp);
}

void BMP280_SetTemperatureOversampling(BMP280_t *bmp, uint8_t TOversampling)
{
	uint8_t Tmp;

	if(TOversampling > 5 ) TOversampling = 5;

	Tmp = Read8(bmp, BMP280_CONTROL);

	Tmp = Tmp & 0x1F; // Tmp (000x xxxx) Mask (0001 1111)
	Tmp |= (TOversampling << 5) & 0xE0; // Mask (1110 000)

	Write8(bmp, BMP280_CONTROL, Tmp);
}

uint32_t BMP280_ReadTemperatureRaw(BMP280_t *bmp)
{
	uint32_t Tmp;
	Tmp = Read24(bmp, BMP280_TEMPDATA);
	Tmp >>= 4;
	return Tmp;
}

uint32_t BMP280_ReadPressureRaw(BMP280_t *bmp)
{
	uint32_t Tmp;
	Tmp = Read24(bmp, BMP280_PRESSUREDATA);
	Tmp >>= 4;
	return Tmp;
}

float BMP280_ReadTemperature(BMP280_t *bmp)
{
	int32_t var1, var2, T;

	int32_t adc_T;

	adc_T = BMP280_ReadTemperatureRaw(bmp);

	var1 = ((((adc_T>>3) - ((int32_t)(bmp->t1)<<1))) * ((int32_t)(bmp->t2))) >> 11;

	var2 = (((((adc_T>>4) - ((int32_t)(bmp->t1))) * ((adc_T>>4) - ((int32_t)(bmp->t1)))) >> 12) *
	((int32_t)(bmp->t3))) >> 14;

	bmp->t_fine = var1 + var2;

	T = ((bmp->t_fine) * 5 + 128) >> 8;

	return (float)(T/100.0);
}

uint8_t BMP280_ReadTemperatureAndPressure(BMP280_t *bmp, float *Temperature, float *Pressure)
{
	float Temp;
	Temp = BMP280_ReadTemperature(bmp);

	int32_t var1, var2;
	uint32_t p;

	int32_t adc_P;

	adc_P = BMP280_ReadPressureRaw(bmp);

	var1 = (((int32_t) bmp->t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) (bmp->p6));
	var2 = var2 + ((var1 * ((int32_t) (bmp->p5))) << 1);
	var2 = (var2 >> 2) + (((int32_t) (bmp->p4)) << 16);
	var1 = ((((bmp->p3) * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) (bmp->p2)) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t) (bmp->p1))) >> 15);

	if (var1 == 0)
	{
		return 1; // avoid exception caused by division by zero
	}

	p = (((uint32_t)(((int32_t) 1048576) - adc_P) - (var2 >> 12))) * 3125;

	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t) var1);
	}
	else
	{
		p = (p / (uint32_t) var1) * 2;
	}
	var1 = (((int32_t) (bmp->p9)) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t) (bmp->p8))) >> 13;

	p = (uint32_t)((int32_t) p + ((var1 + var2 + (bmp->p7)) >> 4));

	*Temperature = Temp;
	*Pressure = (float)(p/100.0);
	return 0;


}

uint8_t BMP280_Init(BMP280_t *bmp, I2C_HandleTypeDef *i2c, uint8_t Address)
{
	uint8_t ChipID;
	bmp->i2c = i2c;
	bmp->Address = Address;


	ChipID = Read8(bmp , BMP280_CHIPID);
	if(ChipID != 0x58)
	{
		return 1;
	}

	bmp->t1 = Read16(bmp, BMP280_DIG_T1);
	bmp->t2 = Read16(bmp, BMP280_DIG_T2);
	bmp->t3 = Read16(bmp, BMP280_DIG_T3);

	bmp->p1 = Read16(bmp, BMP280_DIG_P1);
	bmp->p2 = Read16(bmp, BMP280_DIG_P2);
	bmp->p3 = Read16(bmp, BMP280_DIG_P3);
	bmp->p4 = Read16(bmp, BMP280_DIG_P4);
	bmp->p5 = Read16(bmp, BMP280_DIG_P5);
	bmp->p6 = Read16(bmp, BMP280_DIG_P6);
	bmp->p7 = Read16(bmp, BMP280_DIG_P7);
	bmp->p8 = Read16(bmp, BMP280_DIG_P8);
	bmp->p9 = Read16(bmp, BMP280_DIG_P9);

	BMP280_SetMode(bmp, BMP280_MODE_NORMAL);
	BMP280_SetTemperatureOversampling(bmp, BMP280_TEMPRATURE_20BIT);
	BMP280_SetPressureOversampling(bmp, BMP280_PRESSURE_ULTRAHIGHRES);
	return 0;
}


