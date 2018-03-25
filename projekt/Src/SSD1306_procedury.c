/*
 * SSD1306_procedury.c
 *
 * Procedury sterowania wyœwietlaczem 128x64 SSD1306
 * przystosowane do wspó³pracy z bibliotek¹ EEPROM_procedury.c
 *  Created on: 3 gru 2015
 *      Author: Rysiek www.ars.info.pl
 */
#include "SSD1306_procedury.h"
#include "EEPROM_procedury.h"
#include "tm_stm32_delay.h"
#include "defines.h"

//**********************************************
// procedura inicjacji SSD1306
//wy: status TRUE -sukces, FALSE -b³¹d
char SSD1306_Inicjacja(void)
{
char status=TRUE;

	Inicjacja_EEPROM();//inicjacja procedur I2C

	/* A little delay */
	Delayms(100);

	/* Init LCD */
	status =status & Write_Command(0xAE); //display off
	status =status & Write_Command(0x20); //Set Memory Addressing Mode
	status =status & Write_Command(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	status =status & Write_Command(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	status =status & Write_Command(0xC8); //Set COM Output Scan Direction
	status =status & Write_Command(0x00); //---set low column address
	status =status & Write_Command(0x10); //---set high column address
	status =status & Write_Command(0x40); //--set start line address
	status =status & Write_Command(0x81); //--set contrast control register
	status =status & Write_Command(0xFF);
	status =status & Write_Command(0xA1); //--set segment re-map 0 to 127
	status =status & Write_Command(0xA6); //--set normal display
	status =status & Write_Command(0xA8); //--set multiplex ratio(1 to 64)
	status =status & Write_Command(0x3F); //
	status =status & Write_Command(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	status =status & Write_Command(0xD3); //-set display offset
	status =status & Write_Command(0x00); //-not offset
	status =status & Write_Command(0xD5); //--set display clock divide ratio/oscillator frequency
	status =status & Write_Command(0xF0); //--set divide ratio
	status =status & Write_Command(0xD9); //--set pre-charge period
	status =status & Write_Command(0x22); //
	status =status & Write_Command(0xDA); //--set com pins hardware configuration
	status =status & Write_Command(0x12);
	status =status & Write_Command(0xDB); //--set vcomh
	status =status & Write_Command(0x20); //0x20,0.77xVcc
	status =status & Write_Command(0x8D); //--set DC-DC enable
	status =status & Write_Command(0x14); //
	status =status & Write_Command(0xAF); //--turn on SSD1306 panel

	return status;
}
//**********************************************
// zapis komendy do SSD1306
//we: Command -kod komendy
//wy: status TRUE -sukces, FALSE -b³¹d
char Write_Command(unsigned char Command)
{
char bufor_komendy[2], status;

	bufor_komendy[0] =0x00;	//write command
	bufor_komendy[1] =(char) Command;
	status =EepZapis(SSD1306_SLAVE_ADR, ADRES_I2C_0BAJT, 0, 2, &bufor_komendy[0]);
	return status;
}
//**********************************************
// zapis danej do SSD1306
//we: Data -dana
//wy: status TRUE -sukces, FALSE -b³¹d
char Write_Data(unsigned char Data)
{
char bufor_danych[2], status;

	bufor_danych[0] =0x40;	//write data
	bufor_danych[1] =(char) Data;
	status =EepZapis(SSD1306_SLAVE_ADR, ADRES_I2C_0BAJT, 0, 2, &bufor_danych[0]);
	return status;
}
//********************************************
// fill_Picture
// pokrywanie ca³ego wyœwietlacza kolorem (wzorem bitowym)
//we: fill_Data -wzór bitowy do pokrycia powierzchni
void fill_picture(unsigned char fill_Data)
{
unsigned char m,n;

	for(m=0;m<8;m++)
	{
		Write_Command(0xb0+m);		//page0-page1
		Write_Command(0x00);		//low column start address
		Write_Command(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				Write_Data(fill_Data);
			}
	}
}
//**********************************************
// wyœwietlanie obrazka w formacie XBM 128x64
// we: wskaŸnik do buforu obrazka 128x64 z danymi w formacie XBM
void picture_XBM(const unsigned char *p_obrazek)
{
unsigned char buf_XBM_page[128], buf_konwersja_page[128];

unsigned char x,y, cc;
unsigned int i=0;


  for(y=0;y<8;y++)
  {
	Write_Command(0xb0+y);
	Write_Command(0x00);
	Write_Command(0x10);
	for (cc=0; cc<128; cc++)
	{
		buf_XBM_page[cc] =*(p_obrazek+((y*128)+cc));
	}
	picture_XBM_konwersja(&buf_XBM_page[0], &buf_konwersja_page[0]);
	i=0;
	for(x=0;x<128;x++)
	{
		Write_Data(buf_konwersja_page[i++]);
	}
  }

}
//-------------------------------------------
// konwersja danych obrazka w formacie XBM do wyœwietlenia pojedynczego pasa (page) na SSD1306
//we: p_buf_XBM_page -wskaŸnik do pocz¹tku danych do konwersji
//	  p_buf_konwersja_page -wskaŸnik do buforu dla danych po konwersji
void picture_XBM_konwersja(unsigned char *p_buf_XBM_page, unsigned char *p_buf_konwersja_page)
{
unsigned char bajt_page, bit_bajt_page, bajt_page_linia, konwersja_bajt_licznik;
unsigned char bajt, bajt_konwersja, maska;

	konwersja_bajt_licznik =0;

	for (bajt_page=0; bajt_page<16; bajt_page++)
	{
		maska =0x01;
		for (bit_bajt_page=0; bit_bajt_page<8; bit_bajt_page++)
		{
			bajt_konwersja =0;
			for (bajt_page_linia=0; bajt_page_linia<8; bajt_page_linia++)
			{
				bajt_konwersja =bajt_konwersja >>1;
				bajt =*(p_buf_XBM_page +(bajt_page+(bajt_page_linia*16)));
				if ((bajt &maska) !=0) bajt_konwersja =bajt_konwersja | 0x80;
			}
			*(p_buf_konwersja_page+konwersja_bajt_licznik) = bajt_konwersja;
			konwersja_bajt_licznik++;
			maska =maska <<1;
		}
	}
}
