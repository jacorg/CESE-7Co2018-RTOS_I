/*Using:
 * I2C driver for DS3231 RTC for ESP8266.
 * Copyright 2015 Richard A Burton
 *
 */



#include "sapi_datatypes.h"

// I2C baudrate
#define DS3231_I2C_RATE              400000 // 400 kHz
// DS3231 Address
#define DS3231_ADDR 0x68

#define DS3231_ADDR_TIME    0x00
#define DS3231_ADDR_ALARM1  0x07
#define DS3231_ADDR_ALARM2  0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS  0x0f
#define DS3231_ADDR_AGING   0x10
#define DS3231_ADDR_TEMP    0x11

#define DS3231_SET     0
#define DS3231_CLEAR   1
#define DS3231_REPLACE 2

#define DS3231_12HOUR_FLAG 0x40
#define DS3231_12HOUR_MASK 0x1f
#define DS3231_PM_FLAG     0x20
#define DS3231_MONTH_MASK  0x1f

typedef struct{
				uint8_t tm_sec;
				uint8_t tm_min;
				uint8_t tm_hour;
				uint8_t tm_wday;
				uint8_t tm_mday;
				uint8_t tm_mon;
				uint8_t tm_year;
				uint8_t tm_isdst;

}tm;


static uint8_t decToBcd  (uint8_t dec);
static uint8_t bcdToDec  (uint8_t bcd);
bool_t   ds3231_setTime    (tm *time);
bool_t   ds3231_getTime    (tm *time);
void nameFile(uint8_t *msj, tm *Current_time);
