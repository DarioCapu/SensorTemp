
/*
 * Date: 2016-04-26
 */

/*==================[inclusions]=============================================*/

#include "sapi.h"       // <= sAPI header
#include "sapi_i2c.h"   /* <= sAPI I2C header */
#include "sapi_adc.h"	// sAPI ADC header

/*===========================================================================*/

#define GY906_ADDR	0x00	// Dirección del sensor de temperatura
#define TAMB_ADDR	0x06	// Dirección de la temperatura de ambiente
#define TOBJ1_ADDR	0x07	// Dirección de la temperatura del objeto

/*===========================================================================*/

DEBUG_PRINT_ENABLE;			// Declara debugPrint

/*=========[Función para convertir punto flotante a string]==================*/

static void floatToString( float valor, char *dst, uint8_t pos );

/*=========[MAIN]============================================================*/

int main(void){

	/* ------------- DECLARACIONES --------------- */

	uint16_t t_amb=0;
	uint16_t t_obj1=0;
	float temp,tension;
	uint8_t dato[3];
	uint8_t dataToRead;
	char buffout[64];

   /* ------------- INICIALIZACIONES ------------- */

   boardConfig();	// Inicializar la placa
   debugPrintConfigUart( UART_USB, 115200 );	// Inicializa la comunicación seire
   // Imprimo cartel de inicio
   debugPrintString( "***********************************************************\n\r" );
   debugPrintString( "*          Proyecto - 6648 Sistemas Embebidos             *\n\r" );
   debugPrintString( "***********************************************************\n\r" );
   i2cConfig( I2C0, 100000 );	// Inicializa la comunicación I2C
   adcConfig( ADC_ENABLE );		// Inicializa el ADC

   /* ------------- REPETIR POR SIEMPRE ------------- */
   while(1) {

      /* Prendo el led azul */
      gpioWrite( LEDB, ON );
      delay(500);

      /* Apago el led azul */
      gpioWrite( LEDB, OFF );
      delay(500);

      /* Leo temperatura de ambiente del sensor */
      dataToRead = TAMB_ADDR;
      i2cRead( I2C0,GY906_ADDR,&dataToRead,1,TRUE,&dato,3,TRUE );
      debugPrintString( "Temperatura de ambiente: " );
      t_amb = dato[0];
      t_amb |= dato[1]<<8;
      temp = 0.02 * t_amb - 273.15;
      floatToString( temp, buffout, 0 );
      debugPrintString( buffout );

      debugPrintString( " / " );

      /* Leo temperatura de objeto del sensor */
      dataToRead = TOBJ1_ADDR;
      i2cRead( I2C0,GY906_ADDR,&dataToRead,1,TRUE,&dato,3,TRUE );
      debugPrintString( "Temperatura de objeto: " );
      t_obj1 = dato[0];
      t_obj1 |= dato[1]<<8;
      temp = 0.02 * t_obj1 - 273.15;
      floatToString( temp, buffout, 0 );
      debugPrintString( buffout );

      debugPrintEnter();

      /* Leo la tensión de la entreda analógica */
      tension = adcRead( CH1 ) * 3.30 / 1024 ;
      debugPrintString( "Entrada analógica: " );
      floatToString( tension, buffout, 0 );
      debugPrintString( buffout );

      debugPrintEnter();
   }

   return 0 ;
}


/*=========[Función para convertir punto flotante a string]==================*/

static void floatToString( float valor, char *dst, uint8_t pos ){
	uint16_t val;
	val = 100 * valor;
	dst[pos] = (val / 1000) + '0';
	pos++;
	dst[pos] = (val % 1000) / 100 + '0';
	pos++;
	dst[pos] = '.';
	pos++;
	dst[pos] = (val % 100) / 10 + '0';
	pos++;
	dst[pos] = (val % 10)  + '0';
	pos++;
	dst[pos] = '\0';
}

/*==================[end of file]============================================*/
