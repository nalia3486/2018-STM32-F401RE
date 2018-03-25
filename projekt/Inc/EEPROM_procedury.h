/**
  ******************************************************************************
  * plik    EEPROM_procedury.h
  * funkcja	Plik naglowkowy procedur zwi¹zanych z programow¹ obs³ug¹ pamiêci EEPROM (I2C)
  * autor 	ARIES Ryszard Szymaniak www.arskam.com
  * wersja 	1.0
  * data    3-12-2015
  ******************************************************************************
  */
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __EEPROM_PROCEDURY_H
#define __EEPROM_PROCEDURY_H

#include "stm32f4xx_hal.h"

void Inicjacja_EEPROM(void);
char EepOdczyt(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned int ilosc_bajtow, char *p_bufor);
char EepOdczytBloku(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned char ilosc_bajtow, char *p_bufor);
char EepZapis(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned int ilosc_bajtow, char *p_bufor);
char EepZapisBloku(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned char ilosc_bajtow, char *p_bufor);
unsigned char I2c_(unsigned char ile_bajtow_adresu);
unsigned char I2c_byte_wr(unsigned char bajt);
unsigned char I2c_byte_rd(unsigned char ack_on);
unsigned char I2c_cnl(void);
void I2c_start(void);
void I2c_stop(void);

//definicje linii IO
#define LINIE_EEPn			2
typedef enum
{
	EEP_SDA			=0,
	EEP_SCL			=1
}Linie_EEP_TypeDef;

//LINIA_EEP_SDA linia SDA
#define LINIA_EEP_SDA_PORT			GPIOC
#define LINIA_EEP_SDA_PIN			GPIO_PIN_0
//LINIA_EEP_SCL linia SCL
#define LINIA_EEP_SCL_PORT			GPIOC
#define LINIA_EEP_SCL_PIN			GPIO_PIN_1


#define ADRES_I2C_1BAJT   0
#define ADRES_I2C_2BAJTY  1
#define ADRES_I2C_0BAJT   2

//deklaracje zwiazane z EEPROM-em
#define ROZMIAR_STRONY_EEPROM			130	//rozmiar strony EEPROM w bajtach do zapisu grupowego
#define ROZMIAR_PAMIECI_EEPROM			16384
//rozmiar buforu do operacji zapisu/odczytu EEPROM
#define ROZMIAR_BUF_POSREDNI			0x100




#endif

