# Control de un módulo FPGA para adquisición de señales LiDAR basado en FreeRTOS. 

## Nombre de la organización que propone el Trabajo Final
## Jacobo O. Salvador 
## Datos de contacto
## Jacobo O. Salvador, jacosalvador@gmail.com

#1.Objetivo general
 
Lograr controlar un módulo FPGA el cual está siendo desarrollado como parte del trabajo final de la CESE. Se busca mediante comunicación serie, almacenar información del dispositivo FPGA en una tarjeta microSD con información temporal asociada por medio de un reloj de tiempo real (RTC)  implementado sobre la EDU-CIAA. Se utilizará un sistema operativo de tiempo real (FreeRTOS).

#2.Objetivos especificos 

Utilizar las herramientas ofrecidas por FreeRTOS para implementar diversas tareas que cumplan objetivos claros. El proyecto presenta hardware adicional: módulo SPI para manejo de microSD y un RTC comunicado por I2C. Se utiliza desde FreeRTOS mecanismo de comunicación entre tareas por medio de colas, uso de interrupción para manejo de tecla con semáforo asociado.


