/*
 * ds1307.c
 *
 *  Created on: 18/01/2023
 *      Author: tomed
 */
#include<string.h>
#include  "ds1307.h"


static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value,uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value);
static uint8_t bcd_to_binary(uint8_t value);

I2C_Handle_t g_ds1307I2CHandle;

uint8_t ds1307_init(void)
{

	//initalize the i2c pins
	ds1307_i2c_pin_config();

	//iniatlize the i2c peripheals
	ds1307_i2c_config();

	//enable the i2c peripheal
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	//Make clock halt = 0 in DS peripheal
	ds1307_write(0x00,DS1307_ADDR_SEC);

	//Read back the clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	return (clock_state >> 7);

	//reteruns 1 clock not enabled
	//return 1 clock enabled

}

void ds1307_set_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds, hrs;
	seconds = binary_to_bcd(rtc_time->seconds);
	//Ensure bit 7 is cleared to not turn of clock
	seconds &= ~(1<<7);
	ds1307_write(seconds,DS1307_ADDR_SEC);

	//send minutes
	ds1307_write(binary_to_bcd(rtc_time->minutes),DS1307_ADDR_MIN);

	//set hours
	hrs = binary_to_bcd(rtc_time->hours);
	//Ensure bit 7 is cleared to not turn of clock
	if(rtc_time->time_format == TIME_FORMAT_24HRS)
	{
		hrs &= ~(1 << 6);
	}
	else
	{
		hrs |= (1 << 6);
		hrs =(rtc_time->time_format ==TIME_FORMAT_12HRS_PM ) ? hrs | (1<<5) : hrs & ~(1<<5);
	}
	ds1307_write(hrs,DS1307_ADDR_HRS);
}
void ds1307_get_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds,hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);

	rtc_time->seconds = bcd_to_binary(seconds);
	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	hrs = ds1307_read(DS1307_ADDR_HRS);
	if(hrs & (1<<6))
	{
		//12hr format
		//ever gives 12hr am (0) or 12hr pm (1)
		rtc_time->time_format = !((hrs & (1<< 5)) == 0) ;
		//clear 6th and 5th position
		hrs &= ~(0x3 << 5);
	}
	else
	{
		//24hrs format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}

	rtc_time->hours = bcd_to_binary(hrs);
}

void ds1307_set_current_date(RTC_date_t * rtc_date)
{
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
}
void ds1307_get_current_date(RTC_date_t * rtc_date)
{
	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
}

static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda,0,sizeof(i2c_sda));
	memset(&i2c_scl,0,sizeof(i2c_scl));

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed =GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed =GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);

}

static void ds1307_i2c_config(void)
{
	g_ds1307I2CHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2CHandle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	g_ds1307I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&g_ds1307I2CHandle);
}


static void ds1307_write(uint8_t value,uint8_t reg_addr)
{
	//Sends two bytes of data to enable the clock
	uint8_t tx[2];
	tx[0]=reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&g_ds1307I2CHandle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t data;
	//Data will be read from current position of pointer so data write is used to set pointer position
	I2C_MasterSendData(&g_ds1307I2CHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, 1);
	I2C_MasterReceiveData(&g_ds1307I2CHandle, &data, 1, DS1307_I2C_ADDRESS, 0);

	return data;
}

static uint8_t binary_to_bcd(uint8_t value)
{
	//Ones is bit 0:3, tens is bit 4:6
	uint8_t tens, ones,bcd;
	//If only 1s bcd value is the same as binary
	bcd = value;
	if(value >= 10)
	{
		tens = value/10;
		ones = value %10;
		bcd = (uint8_t)((tens<<4)|ones );
	}
	return bcd;
}

static uint8_t bcd_to_binary(uint8_t value)
{
	//Ones is bit 0:3, tens is bit 4:6
	//This function gets the amount of tens and ones
	uint8_t tens, ones,binary;
	tens = (value >>4);
	ones = (value & 0x0F);
	binary = tens*10 + ones;

	return (uint8_t)binary;
}


