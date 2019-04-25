/*Using:
 * I2C driver for DS3231 RTC for ESP8266.
 * Copyright 2015 Richard A Burton
 *
 */
#include "ds3231.h"
#include "ciaaI2C.h"
#include "sapi_datatypes.h"

// convert normal decimal to binary coded decimal
static uint8_t decToBcd(uint8_t dec)
{
	return(((dec / 10) * 16) + (dec % 10));
}

// convert binary coded decimal to normal decimal
static uint8_t bcdToDec(uint8_t bcd) {
  return(((bcd / 16) * 10) + (bcd % 16));
}

bool_t ds3231_setTime(tm *time) {

	uint8_t RTC_Time[8];

	// start register
	RTC_Time[0] = DS3231_ADDR_TIME;
	// time/date data
	RTC_Time[1] = decToBcd(time->tm_sec);
	RTC_Time[2] = decToBcd(time->tm_min);
	RTC_Time[3] = decToBcd(time->tm_hour);
	RTC_Time[4] = decToBcd(time->tm_wday + 1);
	RTC_Time[5] = decToBcd(time->tm_mday);
	RTC_Time[6] = decToBcd(time->tm_mon + 1);
	RTC_Time[7] = decToBcd(time->tm_year);

	return ciaaI2CWrite(DS3231_ADDR, RTC_Time, sizeof(RTC_Time));

}
// get the time from the rtc, populates a supplied tm struct
// returns true to indicate success
bool_t ds3231_getTime(tm *time)
{

	int loop;
	uint8_t RTC_Hour[7];

	// start register address
	RTC_Hour[0] = DS3231_ADDR_TIME;
	if (!ciaaI2CWrite(DS3231_ADDR, RTC_Hour, 1)) {
		return OFF;
	}

	// read time
	if (!ciaaI2CRead(DS3231_ADDR, RTC_Hour, 7)) {
		return OFF;
	}

	// convert to unix time structure
	time->tm_sec = bcdToDec(RTC_Hour[0]);
	time->tm_min = bcdToDec(RTC_Hour[1]);
	if (RTC_Hour[2] & DS3231_12HOUR_FLAG) {
		// 12h
		time->tm_hour = bcdToDec(RTC_Hour[2] & DS3231_12HOUR_MASK);
		// pm?
		if (RTC_Hour[2] & DS3231_PM_FLAG) time->tm_hour += 12;
	} else {
		// 24h
		time->tm_hour = bcdToDec(RTC_Hour[2]);
	}
	time->tm_wday = bcdToDec(RTC_Hour[3]) - 1;
	time->tm_mday = bcdToDec(RTC_Hour[4]);
	time->tm_mon  = bcdToDec(RTC_Hour[5] & DS3231_MONTH_MASK) - 1;
	time->tm_year = bcdToDec(RTC_Hour[6]);
	time->tm_isdst = 0;

	return ON;

}


void nameFile(uint8_t *msj, tm *Current_time){

	int8_t filename[40]="SDC:/";

	msj[0]='R';
	msj[1]='G';
	msj[2]='L';
	msj[3]='_';
	msj[4]=50;  //year
	msj[5]=48;
	msj[6]=(Current_time->tm_year/10)+48;
	msj[7]=(Current_time->tm_year%10)+48;

	msj[8]=95;   //underscore

	msj[9]=(Current_time->tm_mon/10)+48;  //month
	msj[10]=(Current_time->tm_mon%10)+48;

	msj[11]=95;   //underscore

	msj[12]=(Current_time->tm_mday/10)+48;   //day
	msj[13]=(Current_time->tm_mday%10)+48;

	msj[14]=95;   //underscore

	msj[15]=(Current_time->tm_hour/10)+48;   //hour
	msj[16]=(Current_time->tm_hour%10)+48;

	msj[17]=95;   //underscore

	msj[18]=(Current_time->tm_min/10)+48;   //minutes
	msj[19]=(Current_time->tm_min%10)+48;

	msj[20]=95;   //underscore

	msj[21]=(Current_time->tm_sec/10)+48;   //seconds
	msj[22]=(Current_time->tm_sec%10)+48;
	msj[23]='.';
	msj[24]='t';
	msj[25]='x';
	msj[26]='t';
	msj[27]='\0';
	strcat(filename,msj );
	strcpy(msj,filename);


}
