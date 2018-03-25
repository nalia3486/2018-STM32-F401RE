/*
 * Obrazki_128_64.h
 *
 * Obrazki i procedury demo dla wyœwietlacza SSD1306
 * wspó³pracuj¹ z bibliotekami:
 * 	tm_stm32_ssd1306.c zmodyfikowan¹ przez ARIES
 * 	tm_stm32_fonts.c
 *  Created on: 4 gru 2015
 *      Author: Rysiek www.ars.onfo.pl
 */

#ifndef APPLICATION_PROCEDURY_SSD1306_OBRAZKI_128_64_H_
#define APPLICATION_PROCEDURY_SSD1306_OBRAZKI_128_64_H_

#define avt_logo_bw_width 128
#define avt_logo_bw_height 64
const unsigned char avt_logo_bw_bits[(avt_logo_bw_width*avt_logo_bw_height)/8];
#define motyl_bw_width 128
#define motyl_bw_height 64
const unsigned char motyl_bw_bits[(motyl_bw_width*motyl_bw_height)/8];
#define twarz_bw_width 128
#define twarz_bw_height 64
const unsigned char twarz_bw_bits[(twarz_bw_width*twarz_bw_height)/8];

void Test_TM_Library(void);
void Test2_TM_Library(void);

#endif /* APPLICATION_PROCEDURY_SSD1306_OBRAZKI_128_64_H_ */
