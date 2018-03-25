/*
 * SSD1306_procedury.h
 *
 * Procedury sterowania wyœwietlaczem 128x64 SSD1306
 * przystosowane do wspó³pracy z bibliotek¹ EEPROM_procedury.c
 *  Created on: 3 gru 2015
 *      Rysiek www.ars.info.pl
 */

#ifndef APPLICATION_SSD1306_PROCEDURY_H_
#define APPLICATION_SSD1306_PROCEDURY_H_

//SSD1306 adres slave wyœwietlacza
#define SSD1306_SLAVE_ADR		0x78

char SSD1306_Inicjacja(void);
char Write_Command(unsigned char Command);
char Write_Data(unsigned char Data);

void fill_picture(unsigned char fill_Data);
void picture_XBM(const unsigned char *p_obrazek);
void picture_XBM_konwersja(unsigned char *p_buf_XBM_page, unsigned char *p_buf_konwersja_page);

#endif /* APPLICATION_SSD1306_PROCEDURY_H_ */
