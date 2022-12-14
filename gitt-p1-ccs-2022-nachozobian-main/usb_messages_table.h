/*
 * Listado de los tipos de mensajes empleados en la aplicación, así como definiciones de sus parámetros.
 * (TODO) Incluir aquí las respuestas a los mensajes
*/
#ifndef __USB_MESSAGES_TABLE_H
#define __USB_MESSAGES_TABLE_H

#include<stdint.h>
#include<string.h>
#include<stdbool.h>

//Codigos de los mensajes. EL estudiante deberÃ¡ definir los códigos para los mensajes que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt

typedef enum {
    MENSAJE_NO_IMPLEMENTADO,
    MENSAJE_PING,
    MENSAJE_INYECTORA,
    MENSAJE_GENERADORA,
    MENSAJE_CONSUMIDORA,
    MENSAJE_ALARMA_DEPOSITO,
    MENSAJE_ALARMA_ENERGIA,
    MENSAJE_PAUSA,



    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensajes. El estuadiante debera crear las
// estructuras adecuadas a los mensajes usados, y asegurarse de su compatibilidad con el extremo Qt

#pragma pack(1)   //Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
//Con lo de abajo consigo que el alineamiento de las estructuras en memoria del microcontrolador no tenga relleno
#define PACKED __attribute__ ((packed))

typedef struct {
    uint8_t message;
} PACKED PARAM_MENSAJE_NO_IMPLEMENTADO;

typedef union{
    struct {
        char nombre[6];
        uint32_t eficiencia;
        uint32_t caudal;
    } PACKED InfGas;
    uint8_t  ui8InfGas;
} PACKED PARAM_MENSAJE_INYECTORA;

typedef union{
    struct {
        uint32_t energia;
        uint32_t desecho;
        int32_t energia_consumida;
    } PACKED InfGen;
    uint8_t  ui8InfGen;
} PACKED PARAM_MENSAJE_GENERADORA;

typedef union{
    uint32_t ComenzarTerminar;
    uint8_t  ui8Consumidora;
} PACKED PARAM_MENSAJE_CONSUMIDORA;


typedef union{
    int32_t Tiempo;
    uint8_t  ui8AlarmaDeposito;
} PACKED PARAM_MENSAJE_ALARMA_DEPOSITO;

typedef union{
    int32_t Tiempo;
    uint8_t  ui8AlarmaEnergia;
} PACKED PARAM_MENSAJE_ALARMA_ENERGIA;

typedef union{
    uint32_t ComenzarTerminar;
    uint8_t  ui8Pausa;
} PACKED PARAM_MENSAJE_PAUSA;



//#pragma pack()    //...Pero solo para los mensajes que voy a intercambiar, no para el resto





#endif
