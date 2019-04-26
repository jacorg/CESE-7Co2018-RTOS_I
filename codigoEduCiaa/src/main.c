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
#include "semphr.h"
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

//Interrupcion de teclado
//#include "interrup.h"
/*==================[definiciones y macros]==================================*/
#define N 1024     //el sistema almacena bloques de 256 bytes
#define INIT 0
/*==================[definiciones de datos internos]=========================*/
xQueueHandle queue;
static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file
/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;
SemaphoreHandle_t xSemaphoreTEC1 = NULL;
/*==================[declaraciones de funciones internas]====================*/
void diskTickHook( void *ptr );
// Prototipo de funcion de la tarea
void TaskReadRS232      ( void* taskParmPtr );
void TaskWriteData      ( void* taskParmPtr );
void TaskdiskTickHook   ( void* taskParmPtr );
void tec1               ( void* taskParmPtr );
void initIRQ(void);
void GPIO0_IRQHandler(void);
/*==================[declaraciones de funciones externas]====================*/
 void diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
}

 void initIRQ() {
 	Chip_PININT_Init(LPC_GPIO_PIN_INT);
 	Chip_SCU_GPIOIntPinSel(0, 0, 4); //Mapeo del pin donde ocurrirá el evento y
 	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);//Se configura el canal
 	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);//Se configura para que el
 	NVIC_SetPriority( PIN_INT0_IRQn, 5 );
 	NVIC_EnableIRQ(PIN_INT0_IRQn);
 }

 void GPIO0_IRQHandler(void){
 	portBASE_TYPE xSwitchRequired = pdFALSE;

 	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0){ //Verificamos que la interrupción es la esperada
 		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0); //Borramos el flag de interrupción
 		xSemaphoreGiveFromISR( xSemaphoreTEC1, &xSwitchRequired ); //En este caso libero un semáforo

 	}
 	portEND_SWITCHING_ISR(xSwitchRequired);//Terminar con taskYIELD_FROM_ISR (&xSwitchRequired);
 }





/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
 //--------------------------------------------------------------------------
//--------------------------------main---------------------------------------
//---------------------------------------------------------------------------
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();
   initIRQ();
   xSemaphoreTEC1 = xSemaphoreCreateBinary();
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

      // Crear tarea en freeRTOS
      	xTaskCreate(tec1,                     // Funcion de la tarea a ejecutar
      				(const char *) "tec1", // Nombre de la tarea como String amigable para el usuario
      				configMINIMAL_STACK_SIZE * 2, // Cantidad de stack de la tarea
      				0,                          // Parametros de tarea
      				tskIDLE_PRIORITY + 2,         // Prioridad de la tarea
      				0                         // Puntero a la tarea creada en el sistema
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
/*===========================================================================*/
/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea
void TaskReadRS232( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	uint8_t dato;

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
    		  i++;
    	  }
    	  // Si el buffer interno de recepción se lleno comprimo y copio los datos.
    	  if(i==N)                                  // Buffer lleno entonces vuelco a memoria
    		  if (ds3231_getTime(&Current_time)){   //Lectura DS3231
    		  	 nameFile(msj, &Current_time);      //genero string con formato SSS_AAAA_MM_DD_HH_mm_SS.txt
    		  	 	 	 	 	 	 	 	        // donde SSS:nombre estacion, A:año, M:mes, DD:día, H:hora, mm:minutos, SS:segundos
    		     printf("Archivo a guardar en microSD %s \r\n",msj);

    		    //Creo archivo
    		  if( f_open( &fp, msj, FA_WRITE | FA_OPEN_APPEND ) == FR_OK )
    		     f_write( &fp,dataIn,N, &nbytes );
    		  if( nbytes == N)
    			  gpioToggle(LED2);

    		  f_close(&fp);
    		  i=INIT; // preparo para recibir otro bloque de datos
    		  printf("datos grabados correctamente\n");
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


 void tec1(void* taskParmPtr) {

	 tm Current_time;
	 uint8_t msj[20];

 	portTickType xPeriodicity = 500 / portTICK_RATE_MS;
 	portTickType xLastWakeTime = xTaskGetTickCount();
 	while (1) {
 		if ( xSemaphoreTake( xSemaphoreTEC1, portMAX_DELAY) == pdTRUE) {
 			if (ds3231_getTime(&Current_time)){   //Lectura DS3231
 				infoTime(msj, &Current_time);
 				printf("AAA/MM/DD:HH:mm:SS (RTC)->%s \r\n.",msj);
 			}
 			vTaskDelay(40);
 		}
 		vTaskDelayUntil(&xLastWakeTime, xPeriodicity);
 	}
 }



/*==================[fin del archivo]========================================*/
