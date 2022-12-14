//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Abril2021)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
//  grÃ¡fico (GUI) Qt.
//  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un mensaje
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include<stdbool.h>
#include<stdint.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "utils/cpu_usage.h"
#include "commands.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include "usb_messages_table.h"

#include <string.h>
#include "event_groups.h"
#include "timers.h"

#include "drivers/rgb.h"


//                       TAREAS

#define INTASKPRIO 2 //prioridad de las tareas inyectoras
#define INTASKSTACKSIZE 256//tamaño de la pila de las tareas inyectores
#define GENTASKPRIO 2 //prioridad de la tarea generadora
#define GENTASKSTACKSIZE 256//tamaño de la pila de la tarea generadora
#define CONTASKPRIO 2 //prioridad de la tarea consumidora
#define CONTASKSTACKSIZE 128//tamaño de la pila de la tarea consumidora
#define ALARTASKPRIO 2 //prioridad de la tarea consumidora
#define ALARTASKSTACKSIZE 128//tamaño de la pila de la tarea consumidora


TaskHandle_t Inyectora1Handle;
TaskHandle_t Inyectora2Handle;
TaskHandle_t Inyectora3Handle;
TaskHandle_t GeneradoraHandle;
TaskHandle_t ConsumidoraHandle;
TaskHandle_t AlarmaHandle;


//                      SEMAFOROS

SemaphoreHandle_t mutex_usb;
SemaphoreHandle_t SemaforoEnergia;


//                      COLAS

//TAMAÑOS DE COLA

#define COLAINYECTORA_TAM      3
#define COLACONSUMIDORA_TAM      3
#define COLAMODVAR_TAM      3
#define BINARY_SEMAPHORE_LENGTH 1
#define COMBINED_LENGTH ( COLAINYECTORA_TAM + COLACONSUMIDORA_TAM + COLAMODVAR_TAM + BINARY_SEMAPHORE_LENGTH)

#define COLAMODVARALARMA_TAM      3

#define COLACAUDALGAS_TAM      1
#define COLAMODTIEMPO_TAM      1

//TIPO DE DATOS DE LA COLA

#define ITEM_SIZE_COLAINYECTORA   sizeof( InfGas )
#define ITEM_SIZE_COLACONSUMIDORA   sizeof( uint8_t )
#define ITEM_SIZE_COLAMODVAR   sizeof( ModVar )

#define ITEM_SIZE_COLAMODVARALARMA   sizeof( ModVarAlarma )

#define ITEM_SIZE_COLACAUDALGAS   sizeof( Caudal )
#define ITEM_SIZE_COLAMODTIEMPO   sizeof( uint32_t )

//DEFINICION DE LAS COLAS

static QueueSetHandle_t ColaSet;
QueueHandle_t ColaInyectora, ColaConsumidora , ColaModVar;

QueueHandle_t ColaModVarAlarma;

//QueueHandle_t ColaCaudalArgon,ColaCaudalHelio,ColaCaudalXenon;

QueueHandle_t ColaCaudalGases;

QueueHandle_t ColaModTiempo;


//                      GRUPO DE EVENTOS


static EventGroupHandle_t ConsumirEventos;

#define CONSUMIR_FLAG 0x0002

 EventGroupHandle_t DepuracionEventos;

#define DEPURACION_FLAG 0x0001

 EventGroupHandle_t AlarmaEventos;

#define DEPOSITO_FLAG 0x0001
#define EXCESO_ENERGIA_FLAG 0x0002
#define CERO_ENERGIA_FLAG 0x0004
#define STOP_DEPOSITO_FLAG 0x0008
#define STOP_EXCESO_ENERGIA_FLAG 0x0010
#define MOD_VAR_ALARMA_FLAG 0x0020


 EventGroupHandle_t BotonEventos;
 #define RISING_FLAG 0x0001
 #define CERO_INYECTORA1_FLAG 0x0002
 #define CERO_INYECTORA2_FLAG 0x0004
 #define CERO_INYECTORA3_FLAG 0x0008


 //                   TIPO DE DATOS (typedef)

 typedef struct {

     char nombre[6];
     uint32_t eficiencia;
     uint32_t caudal;

 }InfGas;

 typedef struct {

     int32_t BalanceEnergetico;
     int32_t Desecho;
     int32_t EnergiaConsumida;


 }ModVar;

 typedef struct {

     int32_t TiempoDesecho;
     int32_t TiempoEnergia;

 }ModVarAlarma;

 typedef uint32_t Caudal[3];


 //                         TIMER SOFTWARE

 TimerHandle_t TimerDeposito;
 void vTimerCallbackDeposito();

 TimerHandle_t TimerEnergia;
 void vTimerCallbackEnergia();


 //                         RUTINAS DE INTERRUPCION (RTI)


 void Timer3IntHandler(void);

 void Timer5IntHandler(void);

 //void ADCInterruption0(void);
 void ADCInterruption1(void);
 //void ADCInterruption2(void);

 //                         CONSTANTES

#define CAUDAL_INICIAL 10
#define ENERGIA_MAXIMA 3000
#define DEPOSITO_MAXIMO 1000

#define tiempo_udT 1//tiempo udP



// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;




//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
    static uint8_t ui8Count = 0;

    if (++ui8Count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        ui8Count = 0;
    }
    //return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
    SysCtlSleep();
}


//Esto se ejecuta cada vez que se produce un fallo de asignacio de heap
void vApplicationMallocFailedHook (void)
{
    while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...



static portTASK_FUNCTION( Inyectora1Task, pvParameters ){

    InfGas Argon ={"Argon",30,CAUDAL_INICIAL};
    //strcpy(Argon.nombre,"Argon");
    //Argon.nombre[0]='A';
    //Argon.nombre[1]='r';
    //Argon.nombre[2]='g';
    //Argon.nombre[3]='o';
    //Argon.nombre[4]='n';
    //Argon.caudal=10;
    //Argon.eficiencia=30;

    uint32_t total_caudal=0;

    PARAM_MENSAJE_INYECTORA parametro;
    int32_t i32Numdatos;
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila

    EventBits_t depuracion;
    EventBits_t boton;

    Caudal caudal;
    uint32_t VarCaudal_ant;
    float auxiliar;

    uint32_t N;


    while(1){

        boton= xEventGroupGetBits( BotonEventos );
        if( (boton & CERO_INYECTORA1_FLAG) == CERO_INYECTORA1_FLAG){//RISING EDGE
            total_caudal=0;
            boton= xEventGroupClearBits( BotonEventos ,CERO_INYECTORA1_FLAG );
        }

        if(xQueuePeek(ColaCaudalGases,caudal,0) == pdFALSE){
            auxiliar = CAUDAL_INICIAL;
        }else{
            if( abs(caudal[0] -VarCaudal_ant) > 409){
            auxiliar=CAUDAL_INICIAL*caudal[0]*2/4095;
            VarCaudal_ant=caudal[0];
            }
        }
        total_caudal+=(uint32_t) auxiliar;
        Argon.caudal=(uint32_t) auxiliar;

        xQueueSend(ColaInyectora,&Argon,portMAX_DELAY);//Mecanismo IPC Tarea 2

        //Conexcion USB con Qt

        parametro.ui8InfGas=0;
        strcpy(parametro.InfGas.nombre,Argon.nombre);
        parametro.InfGas.caudal=total_caudal;
        parametro.InfGas.eficiencia=30;
        i32Numdatos=create_frame(pui8Frame,MENSAJE_INYECTORA,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
        if (i32Numdatos>=0)
        {
            xSemaphoreTake(mutex_usb,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);

            depuracion= xEventGroupGetBits( DepuracionEventos );
            if(depuracion == DEPURACION_FLAG){
                UARTprintf("Inyecta Argon\r\n");
            }

            xSemaphoreGive(mutex_usb);
        }

        if(xQueuePeek(ColaModTiempo,&N,0) == pdFALSE){
            N=1;
        }


        vTaskDelay(tiempo_udT*1000/( portTICK_PERIOD_MS * N));//tiempo de espera para enviar la informacion

    }


}

static portTASK_FUNCTION( Inyectora2Task, pvParameters ){


    InfGas Helio ={"Helio",20,10};
    //Helio.nombre[0]='H';
    //Helio.nombre[1]='e';
    //Helio.nombre[2]='l';
    //Helio.nombre[3]='i';
    //Helio.nombre[4]='o';
    //Helio.caudal=10;
    //Helio.eficiencia=20;

    uint32_t total_caudal=0;

    PARAM_MENSAJE_INYECTORA parametro;
    int32_t i32Numdatos;
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila

    EventBits_t depuracion;
    EventBits_t boton;

    Caudal caudal;

    uint32_t VarCaudal_ant;
    float auxiliar;

    uint32_t N;

    while(1){

        boton= xEventGroupGetBits( BotonEventos );
        if( (boton & CERO_INYECTORA2_FLAG) == CERO_INYECTORA2_FLAG){//RISING EDGE
            total_caudal=0;
            xEventGroupClearBits( BotonEventos ,CERO_INYECTORA2_FLAG);
        }

        if(xQueuePeek(ColaCaudalGases,caudal,0) == pdFALSE){
            auxiliar = CAUDAL_INICIAL;
        }else{
            if( abs(caudal[1] -VarCaudal_ant) > 409){
            auxiliar=CAUDAL_INICIAL*caudal[1]*2/4095;
            VarCaudal_ant=caudal[1];
            }
        }

        total_caudal+=(uint32_t) auxiliar;
        Helio.caudal=(uint32_t) auxiliar;

        xQueueSend(ColaInyectora,&Helio,portMAX_DELAY);//Mecanismo IPC Tarea 2

        //Conexcion USB con Qt


        parametro.ui8InfGas=0;
        strcpy(parametro.InfGas.nombre,Helio.nombre);
        parametro.InfGas.caudal=total_caudal;
        parametro.InfGas.eficiencia=20;
        i32Numdatos=create_frame(pui8Frame,MENSAJE_INYECTORA,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

        if (i32Numdatos>=0)
        {
            xSemaphoreTake(mutex_usb,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);

            depuracion= xEventGroupGetBits( DepuracionEventos );
            if(depuracion == DEPURACION_FLAG){
                UARTprintf("Inyecta Argon\r\n");
            }

            xSemaphoreGive(mutex_usb);
        }

        if(xQueuePeek(ColaModTiempo,&N,0) == pdFALSE){
            N=1;
        }


        vTaskDelay(tiempo_udT*1000/( portTICK_PERIOD_MS * N));//tiempo de espera para enviar la informacion
    }

}

static portTASK_FUNCTION( Inyectora3Task, pvParameters ){

    InfGas Xenon ={"Xenon",60,10};
    //Xenon.nombre[0]='X';
    //Xenon.nombre[1]='e';
    //Xenon.nombre[2]='n';
    //Xenon.nombre[3]='o';
    //Xenon.nombre[4]='n';
    //Xenon.caudal=10;
    //Xenon.eficiencia=60;

    uint32_t total_caudal=0;

    PARAM_MENSAJE_INYECTORA parametro;
    int32_t i32Numdatos;
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila

    EventBits_t depuracion;
    EventBits_t boton;

    Caudal caudal;

    uint32_t VarCaudal_ant;
    float auxiliar;

    uint32_t N;

    while(1){
        boton= xEventGroupGetBits( BotonEventos );
        if( (boton & CERO_INYECTORA3_FLAG) == CERO_INYECTORA3_FLAG){//RISING EDGE
            total_caudal=0;
            xEventGroupClearBits( BotonEventos ,CERO_INYECTORA3_FLAG);
        }

        if(xQueuePeek(ColaCaudalGases,caudal,0) == pdFALSE){
            auxiliar = CAUDAL_INICIAL;
        }else{
            if( abs(caudal[2] -VarCaudal_ant) > 409){
            auxiliar=CAUDAL_INICIAL*caudal[2]*2/4095;
            VarCaudal_ant=caudal[2];
            }
        }

        total_caudal+=(uint32_t) auxiliar;
        Xenon.caudal=(uint32_t) auxiliar;

        xQueueSend(ColaInyectora,&Xenon,portMAX_DELAY);//Mecanismo IPC Tarea 2

        //Conexcion USB con Qt

        parametro.ui8InfGas=0;
        strcpy(parametro.InfGas.nombre,Xenon.nombre);
        parametro.InfGas.caudal=total_caudal;
        parametro.InfGas.eficiencia=60;
        i32Numdatos=create_frame(pui8Frame,MENSAJE_INYECTORA,&parametro,sizeof(parametro),MAX_FRAME_SIZE);

        if (i32Numdatos>=0)
        {
            xSemaphoreTake(mutex_usb,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);

            depuracion= xEventGroupGetBits( DepuracionEventos );
            if(depuracion == DEPURACION_FLAG){
                UARTprintf("Inyecta Argon\r\n");
            }

            xSemaphoreGive(mutex_usb);
        }

        if(xQueuePeek(ColaModTiempo,&N,0) == pdFALSE){
            N=1;
        }

        vTaskDelay(tiempo_udT*1000/( portTICK_PERIOD_MS * N));//tiempo de espera para enviar la informacion

    }

}


static portTASK_FUNCTION( GeneradoraTask, pvParameters ){

    uint32_t total_energia=0;
    uint32_t total_desecho=0;
    uint32_t total_energia_consumida=0;

    QueueSetMemberHandle_t cola_aux;

    InfGas GasAux;
    uint8_t consumidor;
    ModVar variables;

    PARAM_MENSAJE_GENERADORA parametro;

    int32_t i32Numdatos;
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila

    EventBits_t depuracion;

    uint32_t N;

    while(1){

        cola_aux= xQueueSelectFromSet ( ColaSet, portMAX_DELAY);
        if(cola_aux==ColaInyectora){
            xQueueReceive( ColaInyectora,&GasAux,portMAX_DELAY);
            total_energia+=GasAux.caudal*GasAux.eficiencia*0.01;
            total_desecho+=GasAux.caudal-GasAux.caudal*GasAux.eficiencia*0.01;


            depuracion= xEventGroupGetBits( DepuracionEventos );
            if(depuracion == DEPURACION_FLAG){
                UARTprintf("Genera Energia y Desecho\r\n");
            }

        }else if(cola_aux==ColaConsumidora){
            xQueueReceive( ColaConsumidora,&consumidor,portMAX_DELAY);
            if(consumidor > total_energia  ){
                total_energia_consumida+=total_energia;

                total_energia=0;
                xEventGroupSetBits(AlarmaEventos,CERO_ENERGIA_FLAG );

            }else{
                total_energia= total_energia - consumidor;

                total_energia_consumida+=consumidor;

            }

            depuracion= xEventGroupGetBits( DepuracionEventos );
            if(depuracion == DEPURACION_FLAG){
                UARTprintf("Consumir Energia\r\n");
            }

        }else if(cola_aux==ColaModVar){
            xQueueReceive( ColaModVar,&variables,portMAX_DELAY);

            if(variables.BalanceEnergetico!=-1) {

                total_energia=(uint32_t)variables.BalanceEnergetico;

                depuracion= xEventGroupGetBits( DepuracionEventos );
                if(depuracion == DEPURACION_FLAG){
                    UARTprintf("Impone Energia\r\n");
                }



            }
            if(variables.Desecho!=-1){

                total_desecho=(uint32_t)variables.Desecho;

                depuracion= xEventGroupGetBits( DepuracionEventos );
                if(depuracion == DEPURACION_FLAG){
                    UARTprintf("Impone Desecho\r\n");
                }


            }

            if(variables.EnergiaConsumida!=-1){

                total_energia_consumida=(uint32_t)variables.EnergiaConsumida;

                depuracion= xEventGroupGetBits( DepuracionEventos );
                if(depuracion == DEPURACION_FLAG){
                    UARTprintf("Impone Energia Consumida\r\n");
                }


            }

        }else if(cola_aux==SemaforoEnergia){
            xSemaphoreTake( SemaforoEnergia, 0 );
            if(total_energia> (ENERGIA_MAXIMA/2)){
                total_energia=total_energia-(ENERGIA_MAXIMA/2);
                total_energia_consumida+=(ENERGIA_MAXIMA/2);
            }else{
                total_energia_consumida+=total_energia;
                total_energia=0;
            }

        }

        parametro.InfGen.energia=total_energia;
        parametro.InfGen.desecho=total_desecho;
        parametro.InfGen.energia_consumida=total_energia_consumida;


        i32Numdatos=create_frame(pui8Frame,MENSAJE_GENERADORA,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
        if (i32Numdatos>=0 )
        {
            xSemaphoreTake(mutex_usb,portMAX_DELAY);
            send_frame(pui8Frame,i32Numdatos);
            xSemaphoreGive(mutex_usb);
        }

        if(xQueuePeek(ColaModTiempo,&N,0) == pdFALSE){
            N=1;
        }

        if(total_desecho>=DEPOSITO_MAXIMO && (xTimerIsTimerActive(TimerDeposito)==pdFALSE)){

            xTimerChangePeriod(TimerDeposito, 1000/( portTICK_PERIOD_MS * N)  ,portMAX_DELAY);

            TimerEnable(TIMER3_BASE, TIMER_A);


        }

        if(total_energia>=ENERGIA_MAXIMA && (xTimerIsTimerActive(TimerEnergia)==pdFALSE)){

            xTimerChangePeriod(TimerEnergia, 1000/( portTICK_PERIOD_MS * N) ,portMAX_DELAY);

            TimerEnable(TIMER3_BASE, TIMER_A);



        }

        if(total_energia<(ENERGIA_MAXIMA*0.8) && (xTimerIsTimerActive(TimerEnergia)==pdTRUE)){

            xEventGroupSetBits(AlarmaEventos, STOP_EXCESO_ENERGIA_FLAG);

        }



    }

}


static portTASK_FUNCTION( ConsumidoraTask, pvParameters ){

    EventBits_t uxBits;
    uint8_t consumidor=25;

    uint32_t N;
    while(1){



        uxBits = xEventGroupWaitBits(ConsumirEventos,CONSUMIR_FLAG,pdFALSE,pdFALSE,portMAX_DELAY);//CUANDO LO RECIBA SE VA A QUEDAR EJECUTANDOSE
        if(uxBits == CONSUMIR_FLAG){
            xQueueSend(ColaConsumidora,&consumidor,portMAX_DELAY);
        }


        if(xQueuePeek(ColaModTiempo,&N,0) == pdFALSE){
             N=1;
         }

        vTaskDelay(1000/ (portTICK_PERIOD_MS * N));//tiempo de espera para enviar la informacion

    }

}


static portTASK_FUNCTION( AlarmaTask, pvParameters ){

    EventBits_t uxBits;
    uint32_t ContadorDeposito=0;
    uint32_t ContadorEnergia=0;

    PARAM_MENSAJE_ALARMA_DEPOSITO parametro_deposito;
    parametro_deposito.ui8AlarmaDeposito=1;

    PARAM_MENSAJE_ALARMA_ENERGIA parametro_energia;
    parametro_energia.ui8AlarmaEnergia=1;

    PARAM_MENSAJE_CONSUMIDORA parametro_consumidora;
    parametro_consumidora.ui8Consumidora=1;
    parametro_consumidora.ComenzarTerminar=1;

    ModVarAlarma auxiliarA;

    int32_t i32Numdatos;
    uint8_t pui8Frame[MAX_FRAME_SIZE];  //Ojo, esto hace que esta tarea necesite bastante pila

    uint32_t N;

    while(1){

        uxBits = xEventGroupWaitBits(AlarmaEventos,DEPOSITO_FLAG | EXCESO_ENERGIA_FLAG | STOP_DEPOSITO_FLAG | STOP_EXCESO_ENERGIA_FLAG
                                     | CERO_ENERGIA_FLAG | MOD_VAR_ALARMA_FLAG ,pdTRUE,pdFALSE,portMAX_DELAY);//CUANDO LO RECIBA SE VA A QUEDAR EJECUTANDOSE

        if(xQueuePeek(ColaModTiempo,&N,0) == pdFALSE){
            N=1;
        }


        switch (uxBits){

            case DEPOSITO_FLAG:

                if(ContadorDeposito<500){
                    ContadorDeposito=ContadorDeposito+10;
                    parametro_deposito.Tiempo=(600-ContadorDeposito);
                }else if(ContadorDeposito==500){
                    ContadorDeposito++;
                    xTimerChangePeriod(TimerDeposito, 100/( portTICK_PERIOD_MS * N) ,portMAX_DELAY);
                    parametro_deposito.Tiempo=(600-ContadorDeposito);
                }else if (ContadorDeposito<600){
                    ContadorDeposito++;
                    parametro_deposito.Tiempo=(600-ContadorDeposito);
                }else{
                    ContadorDeposito=0;
                    xTimerStop( TimerDeposito, 0 );
                    parametro_deposito.Tiempo=0;

                    vTaskSuspend( Inyectora1Handle );
                    vTaskSuspend( Inyectora2Handle );
                    vTaskSuspend( Inyectora3Handle);
                    vTaskSuspend( GeneradoraHandle );
                    vTaskSuspend( ConsumidoraHandle);



                    if(xTimerIsTimerActive(TimerEnergia)!=pdFALSE){
                        xTimerStop( TimerEnergia, 0 );

                    }

                    if(xTimerIsTimerActive(TimerDeposito)!=pdFALSE){
                        xTimerStop( TimerDeposito, 0 );

                    }

                    TimerIntDisable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
                 //   TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                    TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
                    ADCIntDisable(ADC0_BASE,1); // Habilitación del secuenciador dentro del periférico

                    vTaskSuspend( AlarmaHandle);


                }


                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARMA_DEPOSITO,&parametro_deposito,sizeof(parametro_deposito),MAX_FRAME_SIZE);
                if (i32Numdatos>=0 )
                {
                    xSemaphoreTake(mutex_usb,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_usb);
                }



                break;


            case EXCESO_ENERGIA_FLAG:
                if(ContadorEnergia<350){
                   ContadorEnergia=ContadorEnergia+10;
                   parametro_energia.Tiempo=(450-ContadorEnergia);
               }else if(ContadorEnergia==350){
                   ContadorEnergia++;
                   xTimerChangePeriod(TimerEnergia,100/( portTICK_PERIOD_MS * N)  ,portMAX_DELAY);
                   parametro_energia.Tiempo=(450-ContadorEnergia);
               }else if (ContadorEnergia<450){
                   ContadorEnergia++;
                   parametro_energia.Tiempo=(450-ContadorEnergia);
               }else{
                   ContadorEnergia=0;
                   xTimerStop( TimerEnergia, 0 );
                   parametro_energia.Tiempo=0;

                   vTaskSuspend( Inyectora1Handle );
                   vTaskSuspend( Inyectora2Handle );
                   vTaskSuspend( Inyectora3Handle);
                   vTaskSuspend( GeneradoraHandle );
                   vTaskSuspend( ConsumidoraHandle);


                   if(xTimerIsTimerActive(TimerEnergia)!=pdFALSE){
                       xTimerStop( TimerEnergia, 0 );

                   }

                   if(xTimerIsTimerActive(TimerDeposito)!=pdFALSE){
                       xTimerStop( TimerDeposito, 0 );

                   }

                   TimerIntDisable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
                //   TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                   TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
                   ADCIntDisable(ADC0_BASE,1); // Habilitación del secuenciador dentro del periférico

                   vTaskSuspend( AlarmaHandle);

               }


               i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARMA_ENERGIA,&parametro_energia,sizeof(parametro_energia),MAX_FRAME_SIZE);
               if (i32Numdatos>=0 )
               {
                   xSemaphoreTake(mutex_usb,portMAX_DELAY);
                   send_frame(pui8Frame,i32Numdatos);
                   xSemaphoreGive(mutex_usb);
               }


                break;

            case STOP_DEPOSITO_FLAG:
                //le doy al boton
                ContadorDeposito=0;
                xTimerStop( TimerDeposito, 0 );

                parametro_deposito.Tiempo=-1;

                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARMA_DEPOSITO,&parametro_deposito,sizeof(parametro_deposito),MAX_FRAME_SIZE);
                if (i32Numdatos>=0 )
                {
                    xSemaphoreTake(mutex_usb,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_usb);
                }

                ModVar auxiliar;
                auxiliar.BalanceEnergetico=-1;
                auxiliar.Desecho=0;
                auxiliar.EnergiaConsumida=-1;
                xQueueSend(ColaModVar,&auxiliar,portMAX_DELAY);

                if(!xTimerIsTimerActive(TimerEnergia)!=pdFALSE){
                    TimerDisable(TIMER3_BASE, TIMER_A);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

                }


                break;

            case STOP_EXCESO_ENERGIA_FLAG:

                //le doy al boton
                ContadorEnergia=0;
                xTimerStop( TimerEnergia, 0 );

                parametro_energia.Tiempo=-1;

                i32Numdatos=create_frame(pui8Frame,MENSAJE_ALARMA_ENERGIA,&parametro_energia,sizeof(parametro_energia),MAX_FRAME_SIZE);
                if (i32Numdatos>=0 )
                {
                    xSemaphoreTake(mutex_usb,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_usb);
                }


                if(!xTimerIsTimerActive(TimerDeposito)!=pdFALSE){
                    TimerDisable(TIMER3_BASE, TIMER_A);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

                }

                break;

            case CERO_ENERGIA_FLAG:

                xEventGroupClearBits( ConsumirEventos,  CONSUMIR_FLAG);

                i32Numdatos=create_frame(pui8Frame,MENSAJE_CONSUMIDORA,&parametro_consumidora,sizeof(parametro_consumidora),MAX_FRAME_SIZE);
                if (i32Numdatos>=0 )
                {
                    xSemaphoreTake(mutex_usb,portMAX_DELAY);
                    send_frame(pui8Frame,i32Numdatos);
                    xSemaphoreGive(mutex_usb);
                }


                break;



            case MOD_VAR_ALARMA_FLAG:

                xQueueReceive(ColaModVarAlarma,&auxiliarA,portMAX_DELAY);

                if( (auxiliarA.TiempoDesecho!=-1) && (xTimerIsTimerActive(TimerDeposito)!=pdFALSE)){

                    if( (ContadorDeposito<500 ) && ( (600-auxiliarA.TiempoDesecho)>=500)){

                        xTimerChangePeriod(TimerDeposito, 100/( portTICK_PERIOD_MS * N)  ,portMAX_DELAY);

                    }else if((ContadorDeposito>500 ) && ( (600-auxiliarA.TiempoDesecho)<=500)){

                        xTimerChangePeriod(TimerDeposito,1000/( portTICK_PERIOD_MS * N)  ,portMAX_DELAY);

                    }

                    ContadorDeposito=(600-auxiliarA.TiempoDesecho);


                }else if((auxiliarA.TiempoEnergia!=-1)  && (xTimerIsTimerActive(TimerEnergia)!=pdFALSE)){

                    if( (ContadorEnergia<500 ) && ( (600-auxiliarA.TiempoEnergia)>=500)){

                        xTimerChangePeriod(TimerEnergia,100/( portTICK_PERIOD_MS * N)  ,portMAX_DELAY);

                    }else if((ContadorEnergia>500 ) && ( (600-auxiliarA.TiempoEnergia)<=500)){

                        xTimerChangePeriod(TimerEnergia, 1000/( portTICK_PERIOD_MS * N)  ,portMAX_DELAY);

                    }


                    ContadorEnergia=(450-auxiliarA.TiempoEnergia);

                }


                break;

        }






    }

}

//
//*****************************************************************************

//// Codigo para procesar los mensajes recibidos a traves del canal USB

static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

    uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
    int32_t i32Numdatos;
    uint8_t ui8Message;
    void *ptrtoreceivedparam;
    uint32_t ui32Errors=0;

    /* The parameters are not used. */
    ( void ) pvParameters;

    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\nBienvenido a la aplicacion Central de Energia (curso 2021/22)!\n");
    UARTprintf("\nAutores: Jose Carlos Girela Gamez y Juan Ignacio Zobian Massetti ");

    uint32_t timerE=0;
    uint32_t timerD=0;


    for(;;)
    {
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
        if (i32Numdatos>0)
        {	//Si no hay error, proceso la trama que ha llegado.
            i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobaciï¿½n checksum
            if (i32Numdatos<0)
            {
                //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error 
            }
            else
            {
                //El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo mensaje
                ui8Message=decode_message_type(pui8Frame);
                //Obtiene un puntero al campo de parametros y su tamanio.
                i32Numdatos=get_message_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
                switch(ui8Message)
                {
                case MENSAJE_PING :
                    //A un mensaje de ping se responde con el propio mensaje
                    i32Numdatos=create_frame(pui8Frame,ui8Message,0,0,MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        send_frame(pui8Frame,i32Numdatos);
                    }else{
                        //Error de creacion de trama: determinar el error y abortar operacion
                        ui32Errors++;
                        // Procesamiento del error
                        //						// Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
                        //						// tener que copiar y pegar todo en cada operacion de creacion de paquete
                        switch(i32Numdatos){
                        case PROT_ERROR_NOMEM:
                            // Procesamiento del error NO MEMORY
                            break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                            //							// Procesamiento del error STUFFED_FRAME_TOO_LONG
                            break;
                        case PROT_ERROR_MESSAGE_TOO_LONG:
                            //							// Procesamiento del error MESSAGE TOO LONG
                            break;
                        }
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE 
                        }
                        break;
                    }
                    break;


                case  MENSAJE_CONSUMIDORA:
                {
                    PARAM_MENSAJE_CONSUMIDORA parametro;
                    if(check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro), &parametro)>0){

                        if(parametro.ComenzarTerminar==1){
                            xEventGroupSetBits(ConsumirEventos, CONSUMIR_FLAG);
                        }
                        else if(parametro.ComenzarTerminar==0){
                            /* Clear bit 0 and bit 4 in xEventGroup. */
                            xEventGroupClearBits( ConsumirEventos,  CONSUMIR_FLAG);
                        }else{

                        }

                    }
                }
                    break;

                case  MENSAJE_PAUSA:
                {
                    PARAM_MENSAJE_PAUSA parametro;
                    if(check_and_extract_message_param(ptrtoreceivedparam, i32Numdatos, sizeof(parametro), &parametro)>0){

                        if(parametro.ComenzarTerminar==1){
                            vTaskSuspend( Inyectora1Handle );
                            vTaskSuspend( Inyectora2Handle );
                            vTaskSuspend( Inyectora3Handle);
                            vTaskSuspend( GeneradoraHandle );
                            vTaskSuspend( ConsumidoraHandle);
                            vTaskSuspend( AlarmaHandle);

                            if(xTimerIsTimerActive(TimerEnergia)!=pdFALSE){
                                xTimerStop( TimerEnergia, 0 );
                                timerE=1;
                            }

                            if(xTimerIsTimerActive(TimerDeposito)!=pdFALSE){
                                xTimerStop( TimerDeposito, 0 );
                                timerD=1;
                            }

                            TimerIntDisable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
                         //   TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                            TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
                            ADCIntDisable(ADC0_BASE,1); // Habilitación del secuenciador dentro del periférico
                        }
                        else if(parametro.ComenzarTerminar==0){
                            vTaskResume( Inyectora1Handle );
                            vTaskResume( Inyectora2Handle );
                            vTaskResume( Inyectora3Handle);
                            vTaskResume( GeneradoraHandle );
                            vTaskResume( ConsumidoraHandle);
                            vTaskResume( AlarmaHandle);

                            if(timerE==1){
                                xTimerStart( TimerEnergia, 0 );
                                timerE=0;
                            }

                            if(timerD==1){
                                xTimerStart( TimerDeposito, 0 );
                                timerD=0;
                            }


                            TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
                           // TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
                            TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
                            ADCIntEnable(ADC0_BASE,1); // Habilitación del secuenciador dentro del periférico

                        }
                        else {while(1);}
                    }
                }
                    break;
                default:
                {
                    PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                    parametro.message=ui8Message;
                    //El mensaje esta bien pero no esta implementado
                    i32Numdatos=create_frame(pui8Frame,MENSAJE_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        send_frame(pui8Frame,i32Numdatos);
                    }
                    break;
                }
                }// switch
            }
        }else{ // if (ui32Numdatos >0)
            //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            ui32Errors++;
            // Procesamiento del error
        }
    }
}







//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

    //
    // Set the clocking to run at 40 MHz from the PLL.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 4)


    // Get the system clock speed.
    g_ui32SystemClock = SysCtlClockGet();


    //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

    //Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
    ButtonsInit();
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_FALLING_EDGE);
    MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);
    MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
    MAP_IntEnable(INT_GPIOF);


    //CONFIGURACOIN DEL PARAPDEO LED DE ALARMAS


    // Configuracion de puerto GPIOF (LEDs)
    // Habilita Puerto F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Configura pines PF1, PF2, y PF3 como salidas (control de LEDs)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    // Y apaga los LEDs
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

    // Configuracion TIMER3
    // Habilita periferico Timer3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER3);
    // Configura el Timer3 para cuenta periodica de 32 bits (no lo separa en TIMER0A y TIMER0B)
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    // Periodo de cuenta de 0.05s. SysCtlClockGet() te proporciona la frecuencia del reloj del sistema, por lo que una cuenta
    // del Timer a SysCtlClockGet() tardara 1 segundo, a 0.5*SysCtlClockGet(), 0.5seg, etc...
    uint32_t ui32Period = (SysCtlClockGet() / 1) / 2;
    // Carga la cuenta en el Timer3A
    TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period -1);
    IntPrioritySet(INT_TIMER3A, 0);
    //Asocia una RTI al TIMER3 antes de habilitarlo
    IntRegister(INT_TIMER3A, Timer3IntHandler);
    // Habilita interrupcion del modulo TIMER
    IntEnable(INT_TIMER3A);
    // Y habilita, dentro del modulo TIMER3, la interrupcion de particular de "fin de cuenta"
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    // Activa el Timer3A (empezara a funcionar)
    //TimerEnable(TIMER3_BASE, TIMER_A);


    //CONFIGURACION DEL ADC

    // Configuracion TIMER4
    // Habilita periferico Timer4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER4);
    // Configura el Timer4 para cuenta periodica de 32 bits (no lo separa en TIMER4A y TIMER4B)
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    // SysCtlClockGet() da el numero de ciclos para 1 s
    uint32_t ui32PeriodADC = SysCtlClockGet() / 5 ;
    // Carga la cuenta
    TimerLoadSet(TIMER4_BASE, TIMER_A, ui32PeriodADC -1);
    // Activa el Timer4A (empezara a funcionar)
    TimerEnable(TIMER4_BASE, TIMER_A);

    //Configuracion pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_1);

    //Configuracion ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // Habilita ADC0
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

    //ADCSequenceDisable(ADC0_BASE, 0); // Deshabilita el secuenciador 0 del ADC0 para su configuracion
    ADCSequenceDisable(ADC0_BASE, 1); // Deshabilita el secuenciador 1 del ADC0 para su configuracion
    //ADCSequenceDisable(ADC0_BASE, 2); // Deshabilita el secuenciador 2 del ADC0 para su configuracion

    ADCHardwareOversampleConfigure(ADC0_BASE,64);//promedio de 64 muestras
    //el timer para configurar el trigger del ADC1
    TimerControlTrigger(TIMER4_BASE,TIMER_A,true);

    //ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);//configura que el trigger sea el timer en el secuenciador 0
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);//configura que el trigger sea el timer en el secuenciador 0
    //ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0);//configura que el trigger sea el timer en el secuenciador 0

    // El conversor lee y  se genera un aviso de interrupcion
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 );
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 );
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
    //ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

    // Tras configurar el secuenciador, se vuelve a habilitar
    //ADCSequenceEnable(ADC0_BASE, 0);
    ADCSequenceEnable(ADC0_BASE, 1);
    //ADCSequenceEnable(ADC0_BASE, 2);

    //interrupciones
    /*IntRegister(INT_ADC0SS0, ADCInterruption0);
    IntEnable(INT_ADC0SS0); // Habilitación a nivel global del sistema
    ADCIntRegister(ADC0_BASE,0, ADCInterruption0);
    ADCIntClear(ADC0_BASE,0); // Borramos posibles interrupciones pendientes
    ADCIntEnable(ADC0_BASE,0); // Habilitación del secuenciador dentro del periférico*/

    IntRegister(INT_ADC0SS1, ADCInterruption1);
    IntEnable(INT_ADC0SS1); // Habilitación a nivel global del sistema
    ADCIntRegister(ADC0_BASE,1, ADCInterruption1);
    ADCIntClear(ADC0_BASE,1); // Borramos posibles interrupciones pendientes
    ADCIntEnable(ADC0_BASE,1); // Habilitación del secuenciador dentro del periférico

    /*IntRegister(INT_ADC0SS2, ADCInterruption2);
    IntEnable(INT_ADC0SS2); // Habilitación a nivel global del sistema
    ADCIntRegister(ADC0_BASE,2, ADCInterruption2);
    ADCIntClear(ADC0_BASE,2); // Borramos posibles interrupciones pendientes
    ADCIntEnable(ADC0_BASE,2); // Habilitación del secuenciador dentro del periférico*/


    //TIMER BOTON MULTIFUNCION


    // Configuracion TIMER5
    // Habilita periferico Timer5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER5);
    // Configura el Timer5 para cuenta periodica de 32 bits (no lo separa en TIMER0A y TIMER0B)
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    // Periodo de cuenta de 0.05s. SysCtlClockGet() te proporciona la frecuencia del reloj del sistema, por lo que una cuenta
    // del Timer a SysCtlClockGet() tardara 1 segundo, a 0.5*SysCtlClockGet(), 0.5seg, etc...
    uint32_t ui32PeriodMultiFuncion = 2*SysCtlClockGet() ;
    // Carga la cuenta en el Timer5A
    TimerLoadSet(TIMER5_BASE, TIMER_A, ui32PeriodMultiFuncion -1);
    IntPrioritySet(INT_TIMER5A, 0);
    //Asocia una RTI al TIMER5 antes de habilitarlo
    IntRegister(INT_TIMER5A, Timer5IntHandler);
    // Habilita interrupcion del modulo TIMER
    IntEnable(INT_TIMER5A);
    // Y habilita, dentro del modulo TIMER5, la interrupcion de particular de "fin de cuenta"
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    // Activa el Timer5A (empezara a funcionar)
    //TimerEnable(TIMER5_BASE, TIMER_A);




    /**                                              Creacion de tareas 									**/

    // Inicializa el sistema de depuraciï¿½n por terminal UART
    if (initCommandLine(512,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }

    USBSerialInit(32,32);	//Inicializo el  sistema USB
    //
    // Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
    //
    if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
    {
        while(1);
    }

    //
    // A partir de aquï¿½ se crean las tareas de usuario, y los recursos IPC que se vayan a necesitar
    //


    if(xTaskCreate(Inyectora1Task,"Inyectora1",INTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + INTASKPRIO, &Inyectora1Handle) != pdTRUE)
    {
        while(1);
    }

    if(xTaskCreate(Inyectora2Task,"Inyectora2",INTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + INTASKPRIO, &Inyectora2Handle) != pdTRUE)
    {
        while(1);
    }

    if(xTaskCreate(Inyectora3Task,"Inyectora3",INTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + INTASKPRIO, &Inyectora3Handle) != pdTRUE)
    {
        while(1);
    }

    if(xTaskCreate(GeneradoraTask,"Generadora",GENTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + GENTASKPRIO, &GeneradoraHandle) != pdTRUE)
    {
        while(1);
    }

    if(xTaskCreate(ConsumidoraTask,"Consumidora",CONTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + CONTASKPRIO, &ConsumidoraHandle) != pdTRUE)
    {
        while(1);
    }

   if(xTaskCreate(AlarmaTask,"Alarma",ALARTASKSTACKSIZE, NULL, tskIDLE_PRIORITY + ALARTASKPRIO, &AlarmaHandle) != pdTRUE)
    {
        while(1);
    }



    //Crea los semáforos compartidos por tareas e ISR
    mutex_usb=xSemaphoreCreateMutex();
    if ((NULL == mutex_usb)){
        while (1); //No hay memoria para los semaforo
    }

    SemaforoEnergia= xSemaphoreCreateBinary(); //crea un semaforo binario
    if ((NULL == SemaforoEnergia))
    {
        while (1); //No hay memoria para los semaforo
    }

    //Crea el grupo de eventos
    ConsumirEventos = xEventGroupCreate();
    if( NULL ==  ConsumirEventos)
    {
        while(1);
    }
    DepuracionEventos = xEventGroupCreate();
    if( NULL ==  DepuracionEventos)
    {
        while(1);
    }
    AlarmaEventos = xEventGroupCreate();
    if( NULL ==  AlarmaEventos)
    {
        while(1);
    }
    BotonEventos = xEventGroupCreate();
    if( NULL ==  BotonEventos)
    {
        while(1);
    }
    ColaSet = xQueueCreateSet( COMBINED_LENGTH );
    if(ColaSet==NULL){
        while(1);
    }
    ColaInyectora = xQueueCreate(COLAINYECTORA_TAM, ITEM_SIZE_COLAINYECTORA );
    if(ColaInyectora==NULL){
        while(1);
    }
    ColaConsumidora= xQueueCreate( COLACONSUMIDORA_TAM, ITEM_SIZE_COLACONSUMIDORA);
    if(ColaConsumidora==NULL){
        while(1);
    }
    ColaModVar= xQueueCreate( COLAMODVAR_TAM, ITEM_SIZE_COLAMODVAR);
    if(ColaModVar==NULL){
        while(1);
    }

    xQueueAddToSet( ColaInyectora, ColaSet );
    xQueueAddToSet( ColaConsumidora, ColaSet);
    xQueueAddToSet( ColaModVar, ColaSet);
    xQueueAddToSet(SemaforoEnergia, ColaSet);

    ColaModVarAlarma= xQueueCreate( COLAMODVARALARMA_TAM, ITEM_SIZE_COLAMODVARALARMA);
    if(ColaModVarAlarma==NULL){
        while(1);
    }


    ColaCaudalGases= xQueueCreate( COLACAUDALGAS_TAM, ITEM_SIZE_COLACAUDALGAS);
    if(ColaCaudalGases==NULL){
        while(1);
    }

    ColaModTiempo= xQueueCreate( COLAMODTIEMPO_TAM, ITEM_SIZE_COLAMODTIEMPO);
    if(ColaModTiempo==NULL){
        while(1);
    }

    // Create and start timer SW
    TimerDeposito = xTimerCreate("TimerSW", configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallbackDeposito);
    if( NULL == TimerDeposito )
     {
         /* The timer was not created. */
        while(1);
     }
    TimerEnergia = xTimerCreate("TimerSW", configTICK_RATE_HZ, pdTRUE,NULL,vTimerCallbackEnergia);
    if( NULL == TimerEnergia )
     {
         /* The timer was not created. */
        while(1);
     }



    //
    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    //
    vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
    while(1)
    {
        //Si llego aqui es que algo raro ha pasado
    }
}



void vTimerCallbackDeposito( TimerHandle_t pxTimer )
{


    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( pxTimer );

    xEventGroupSetBits(AlarmaEventos, DEPOSITO_FLAG);


}

void vTimerCallbackEnergia( TimerHandle_t pxTimer )
{


    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( pxTimer );

    xEventGroupSetBits(AlarmaEventos, EXCESO_ENERGIA_FLAG);


}



// Rutinas de interrupcion
void GPIOFIntHandler(void){

    EventBits_t boton;

    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    BaseType_t higherPriorityTaskWoken=pdFALSE;
    BaseType_t HigherPriorityTaskWoken1=pdFALSE;

    int32_t i32PinStatus=MAP_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);

    if (!(i32PinStatus & LEFT_BUTTON))
    {
        xEventGroupSetBitsFromISR(AlarmaEventos, STOP_DEPOSITO_FLAG, &xHigherPriorityTaskWoken );

    }else {

        boton= xEventGroupGetBitsFromISR( BotonEventos );
        if((boton & RISING_FLAG) == RISING_FLAG ){//RISING EDGE

            xSemaphoreGiveFromISR(SemaforoEnergia,&higherPriorityTaskWoken);

            xEventGroupClearBitsFromISR(BotonEventos, RISING_FLAG );
            TimerDisable(TIMER5_BASE, TIMER_A);
            MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, RIGHT_BUTTON,GPIO_FALLING_EDGE);
        }else if ((boton & RISING_FLAG) != RISING_FLAG ){//FALLING EDGE

            uint32_t ui32PeriodMultiFuncion = 2*SysCtlClockGet() ;
            // Carga la cuenta en el Timer0A
            TimerLoadSet(TIMER5_BASE, TIMER_A, ui32PeriodMultiFuncion -1);
            TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
            TimerEnable(TIMER5_BASE, TIMER_A);
            xEventGroupSetBitsFromISR(BotonEventos, RISING_FLAG, &HigherPriorityTaskWoken1);
            MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, RIGHT_BUTTON,GPIO_RISING_EDGE);

        }


    }

    MAP_GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(HigherPriorityTaskWoken1);

}

void Timer5IntHandler(void)
{
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    BaseType_t xHigherPriorityTaskWoken=pdFALSE;

    EventBits_t boton;
    boton= xEventGroupGetBitsFromISR( BotonEventos );
    xEventGroupSetBitsFromISR(BotonEventos, boton | CERO_INYECTORA1_FLAG | CERO_INYECTORA2_FLAG | CERO_INYECTORA3_FLAG, &xHigherPriorityTaskWoken);

    ModVar auxiliar;
    auxiliar.BalanceEnergetico=0;
    auxiliar.Desecho=0;
    auxiliar.EnergiaConsumida=0;
    xQueueSendFromISR(ColaModVar,&auxiliar, &xHigherPriorityTaskWoken);

    xEventGroupSetBitsFromISR(AlarmaEventos, STOP_DEPOSITO_FLAG, &xHigherPriorityTaskWoken );
    xEventGroupSetBitsFromISR(AlarmaEventos, STOP_EXCESO_ENERGIA_FLAG, &xHigherPriorityTaskWoken );


    xEventGroupClearBitsFromISR(BotonEventos, RISING_FLAG);
    TimerDisable(TIMER5_BASE, TIMER_A);
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, RIGHT_BUTTON,GPIO_FALLING_EDGE);


    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

void Timer3IntHandler(void)
{
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    // Lee el estado actual del LED y escribe el estado opuesto
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    }
}


void ADCInterruption1(void){

     BaseType_t xHigherPriorityTaskWoken=pdFALSE;

     uint32_t ui32ADC0Value[4];
     static  Caudal caudal;
     // Limpia el flag de interrupción del ADC
     ADCIntClear(ADC0_BASE, 1);
     // Leemos los datos del secuenciador a un array
     ADCSequenceDataGet(ADC0_BASE, 1,  ui32ADC0Value);
     caudal[0]=ui32ADC0Value[0];
     caudal[1]=ui32ADC0Value[1];
     caudal[2]=ui32ADC0Value[2];

     //poner las tres colas mejor
     xQueueOverwriteFromISR(ColaCaudalGases, caudal, &xHigherPriorityTaskWoken );

     portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}


