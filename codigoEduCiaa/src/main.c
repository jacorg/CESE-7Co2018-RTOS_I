/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

// sAPI header
#include "sapi.h"
//Strings
#include  "string.h"
//SPI
#include "../../sd_card/inc/sd_spi.h"   // <= own header (optional)
#include "ff.h"                         // <= Biblioteca FAT FS
#include "fssdc.h"

// DS3231
#include  "ciaaI2C.h"
#include  "ds3231.h"

//algoritmos de compresion de datos lz4
#include "lz4.h"

/*==================[definiciones y macros]==================================*/
#define N 1024     //el sistema almacena bloques de 256 bytes
#define INIT 0
/*==================[definiciones de datos internos]=========================*/
xQueueHandle queue;
static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file
/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
void diskTickHook( void *ptr );
// Prototipo de funcion de la tarea
void TaskReadRS232      ( void* taskParmPtr );
void TaskWriteData      ( void* taskParmPtr );
void TaskdiskTickHook   ( void* taskParmPtr );
/*==================[declaraciones de funciones externas]====================*/
 void diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
}
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   /* Inicializar UART_USB a 115200 baudios */
   uartConfig( UART_USB, 115200 );
   // UART for debug messages
   //debugPrintConfigUart( UART_USB, 115200 );
   //debugPrintlnString( "Blinky con freeRTOS y sAPI." );

   // using I2C for communication
   // starting the I2C bus
   ciaaI2CInit();

   //Create the queue
   queue = xQueueCreate(N, sizeof(uint8_t));

// Crear tarea en freeRTOS recepción de caracteres por medio de la UART
   xTaskCreate(
	  TaskReadRS232,                          // Funcion de la tarea a ejecutar
      (const char *)"TaskReadRS232",          // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2,             // Cantidad de stack de la tarea
      0,                                      // Parametros de tarea
      tskIDLE_PRIORITY+3,                     // Prioridad de la tarea
      0                                       // Puntero a la tarea creada en el sistema
   );

// Crear tarea en freeRTOS controlador de memoria SD y RTC
      xTaskCreate(
         TaskWriteData,                       // Funcion de la tarea a ejecutar
         (const char *)"TaskWriteData",       // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2,          // Cantidad de stack de la tarea
         0,                                   // Parametros de tarea
         tskIDLE_PRIORITY+3,                  // Prioridad de la tarea
         0                                    // Puntero a la tarea creada en el sistema
      );

      // Crear tarea en freeRTOS controlador de escritura en SD
      xTaskCreate(
       TaskdiskTickHook,                       // Funcion de la tarea a ejecutar
          (const char *)"TaskdiskTickHook",    // Nombre de la tarea como String amigable para el usuario
          configMINIMAL_STACK_SIZE*2,          // Cantidad de stack de la tarea
          0,                                   // Parametros de tarea
          tskIDLE_PRIORITY+3,                  // Prioridad de la tarea
          0                                    // Puntero a la tarea creada en el sistema
       );

   // Iniciar scheduler
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea
void TaskReadRS232( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
  // printf( "Blinky con freeRTOS y sAPI.\r\n" );

	uint8_t dato;
   // Envia la tarea al estado bloqueado durante 1 s (delay)
   // vTaskDelay( 1000 / portTICK_RATE_MS );
   // gpioWrite( LED1, OFF );

   // Tarea periodica cada 5 ms
   portTickType xPeriodicity =  5 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {


	 if(uartReadByte( UART_USB, &dato )){
	 	   xQueueSendToBack(queue, &dato, portMAX_DELAY);
	 	   gpioToggle( LEDB);
	 }


     // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
     vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}

 void TaskWriteData( void* taskParmPtr )
  {
     // ---------- CONFIGURACIONES ------------------------------
	 // SPI configuration
	 spiConfig( SPI0 );
	 // ------ PROGRAMA QUE ESCRIBE EN LA SD -------
	 UINT nbytes;
	 // Initialize SD card driver
	 FSSDC_InitSPI ();
	 // Give a work area to the default drive
	 if( f_mount( &fs, "SDC:", 0 ) != FR_OK ) {
	 // If this fails, it means that the function could
	 // not register a file system object.
	 // Check whether the SD card is correctly connected
	 }

	 int din,ret,desSize=N;
	 uint16_t i=0;
	 uint16_t j=0;
	 uint8_t datoRcv;

	 uint8_t * dataIn = malloc(N * sizeof(uint8_t)); //asignación de memoria dinámica para
                                                     //recepción de caracteres x queue.
	 uint8_t * dataOut = malloc(N * sizeof(uint8_t));

	 tm Current_time;
	 uint8_t msj[40];   //msj to save the filename

      // Tarea periodica cada 5 ms
      portTickType xPeriodicity =  5 / portTICK_RATE_MS;
      portTickType xLastWakeTime = xTaskGetTickCount();

      // ---------- REPETIR POR SIEMPRE --------------------------
      while(TRUE) {

    	  //Recibe los bytes y los almacena en un array de 1024 valores.
    	  // los datos vienen por un mecanismo de cola "queue"
    	  if (xQueueReceive(queue, &datoRcv, portMAX_DELAY) == pdTRUE) {
    		  *(dataIn+i)=datoRcv;
    		  gpioToggle( LED3);
    		  printf("guardoooo i:%d dato:%d \r\n",i,dataIn[i]);
    		  i++;
    	  }

    	  // Si el buffer interno de recepción se lleno comprimo y copio los datos.
    	  if(i==N)                                  // Buffer lleno entonces vuelco a memoria
    		  if (ds3231_getTime(&Current_time)){   //Lectura DS3231
    		  	 nameFile(msj, &Current_time);      //genero string con formato SSS_AAAA_MM_DD_HH_mm_SS.txt
    		  	 	 	 	 	 	 	 	        // donde SSS:nombre estacion, A:año, M:mes, DD:día, H:hora, mm:minutos, SS:segundos
    		     printf("Archivo a guardar en microSD %s \r\n",msj);

    		   //inicializo algoritmo de compresión de datos para almacenar en microSD
               //int LZ4_compress_destSize(const char* src, char* dst, int* srcSizePtr, int targetDstSize)
    		   //  din=sizeof(dataIn);
    		   //  ret= LZ4_compress_fast (dataIn, dataOut,N, N,1);
    		   //  printf("retorno %d\n",ret);

    		  if( f_open( &fp, msj, FA_WRITE | FA_OPEN_APPEND ) == FR_OK )
    		     f_write( &fp,dataIn,N, &nbytes );
    		  if( nbytes == N)
    			  gpioToggle(LED2);

    		  f_close(&fp);
    		  i=INIT;
    		  printf("datos grabados\n");
    		  }// end ds3231


         // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
         vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
      } //while
}// Task

 void TaskdiskTickHook( void* taskParmPtr )
   {
       // ---------- CONFIGURACIONES ------------------------------



       // Tarea periodica cada 10 ms
       portTickType xPeriodicity =  10 / portTICK_RATE_MS;
       portTickType xLastWakeTime = xTaskGetTickCount();

       // ---------- REPETIR POR SIEMPRE --------------------------
       while(TRUE) {

    	   disk_timerproc();   // Disk timer process
          // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
          vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
       }
 }
/*==================[fin del archivo]========================================*/
