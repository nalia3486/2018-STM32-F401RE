/**
  ******************************************************************************
  * plik    EEPROM_procedury.c
  * funkcja	Plik procedur zwi¹zanych z programow¹ obs³ug¹ pamiêci EEPROM (I2C)
  * autor 	ARIES Ryszard Szymaniak www.arskam.com
  * wersja 	1.0
  * data    3-12-2015
  ******************************************************************************
  */
#include "EEPROM_procedury.h"
#include "defines.h"
#include "tm_stm32_delay.h"

struct {
         unsigned char adres;
         unsigned char subadres;
         unsigned char subadres_second;
         unsigned char ile;
         char *p_bufor;
       }i2c;
unsigned char i2c_r_w;

const uint16_t LINIA_EEP_PIN[LINIE_EEPn]	  ={
		LINIA_EEP_SDA_PIN, LINIA_EEP_SCL_PIN};

GPIO_TypeDef*  LINIA_EEP_PORT[LINIE_EEPn]  ={
		LINIA_EEP_SDA_PORT, LINIA_EEP_SCL_PORT};

char bufor_posredni[ROZMIAR_BUF_POSREDNI];

void GPIO_EEP_Init(void);
void GPIO_EEP_High(Linie_EEP_TypeDef Linia);
void GPIO_EEP_Low(Linie_EEP_TypeDef Linia);
char GPIO_EEP_StanLinii(Linie_EEP_TypeDef Linia);

//-------------------------------------
//inicjacja linii sterujacych EEPROM-em
void Inicjacja_EEPROM(void)
{
  /* Init delay */
  TM_DELAY_Init();

  GPIO_EEP_Init();
  I2c_stop();//stan STOP na liniach SCL i SDA I2c
}
//-----------------
//odczyt z EEPROM-u
char EepOdczyt(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned int ilosc_bajtow, char *p_bufor)
//we:	slave_adr -g³ówny adres uk³adu I2C
//		ile_bajtow_adresu -ile bajtów adresu pocz¹tku danych do odczytu/zapisu
//		adres_int -adres pocz¹tku danych do odczytu
//		ilosc_bajtow -ile bajtów do odczytu
//		p_bufor -wskaŸnik buforu dla danych odczytanych
{
char status=TRUE;
unsigned char ile_do_odczytu;
/*
	while(ilosc_bajtow !=0)
	{
		status =EepOdczytBloku(adres_int, 1, p_bufor);
		if (status !=TRUE) break;
		adres_int++;
		p_bufor++;
		ilosc_bajtow--;
	}
	*/

	while (ilosc_bajtow !=0)
	{
		if (ilosc_bajtow <=ROZMIAR_STRONY_EEPROM) ile_do_odczytu =(0xFF &ilosc_bajtow);
		else ile_do_odczytu =ROZMIAR_STRONY_EEPROM;
		status =EepOdczytBloku(slave_adr, ile_bajtow_adresu, adres_int, ile_do_odczytu, p_bufor);
		adres_int =(adres_int +ile_do_odczytu);
		p_bufor =(p_bufor +ile_do_odczytu);
		ilosc_bajtow =(ilosc_bajtow -ile_do_odczytu);	
	}  
	return status;
}
//-----------------------------------------
//procedura odczytu bloku danych z EEPROM-u
char EepOdczytBloku(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned char ilosc_bajtow, char *p_bufor)
{
//wy: 0 -b³¹d, 1 -sukces procedury
char status;

  i2c.adres =slave_adr;
  i2c.subadres =(unsigned char)(adres_int >>8);
  i2c.subadres_second =(unsigned char)adres_int;  
  i2c.p_bufor =p_bufor;
  i2c.ile =ilosc_bajtow;
  i2c_r_w =1; 
  status =I2c_(ile_bajtow_adresu);
  return status;
}  
//-----------------
//zapis do EEPROM-u
char EepZapis(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned int ilosc_bajtow, char *p_bufor)
//we:	slave_adr -g³ówny adres uk³adu I2C
//		ile_bajtow_adresu -ile bajtów adresu pocz¹tku danych do zapisu
//		adres_int -adres pocz¹tku danych do zapisu
//		ilosc_bajtow -ile bajtów do zapisu
//		p_bufor -wskaŸnik buforu dla danych do zapisu
{
char status=TRUE;
unsigned char ile_do_zapisu, ile_do_zapisu_na_stronie;

/*
	while(ilosc_bajtow !=0)
	{
		status =EepZapisBloku(adres_int, 1, p_bufor);
		if (status !=TRUE) break;
		adres_int++;
		p_bufor++;
		ilosc_bajtow--;
	}
	*/

	while (ilosc_bajtow !=0)
	{
	    ile_do_zapisu_na_stronie =(ROZMIAR_STRONY_EEPROM -(adres_int &(ROZMIAR_STRONY_EEPROM-1)));
		if (ilosc_bajtow <=ile_do_zapisu_na_stronie) ile_do_zapisu =(0xFF &ilosc_bajtow);
		else ile_do_zapisu =ile_do_zapisu_na_stronie;
		status =EepZapisBloku(slave_adr, ile_bajtow_adresu, adres_int, ile_do_zapisu, p_bufor);
		if (status !=TRUE) break;
		adres_int =(adres_int +ile_do_zapisu);
		p_bufor =(p_bufor +ile_do_zapisu);
		ilosc_bajtow =(ilosc_bajtow -ile_do_zapisu);	
	}  
	return status;
}
//-----------------------------------------
//procedura zapisu bloku danych do EEPROM-u
char EepZapisBloku(unsigned char slave_adr, unsigned char ile_bajtow_adresu,
		unsigned int adres_int, unsigned char ilosc_bajtow, char *p_bufor)
{
//wy: 0 -b³¹d, 1 -sukces procedury
unsigned char status;

  i2c.adres =slave_adr;
  i2c.subadres =(unsigned char)(adres_int >>8);
  i2c.subadres_second =(unsigned char)adres_int;  
  i2c.p_bufor =p_bufor;
  i2c.ile =ilosc_bajtow;
  i2c_r_w =0; 
  status =I2c_(ile_bajtow_adresu);
//  HAL_Delay(10);//pauza 10ms
  return status;
}
//--------------------------
//glowna procedura I2C
unsigned char I2c_(unsigned char ile_bajtow_adresu)
//wy: 0 -blad, 1 -sukces procedury
{
unsigned char x;
unsigned char test,ack;

  while (1 !=2)
  {
    test =I2c_cnl();
    if (test ==1) break;//blad
	Delay(5);
    I2c_start();
    test =I2c_byte_wr(i2c.adres);
    if (test ==1) break;//blad
    if (ile_bajtow_adresu !=ADRES_I2C_0BAJT)
    {
      test =I2c_byte_wr(i2c.subadres);
    }
    if (test ==1) break;//blad
    if (ile_bajtow_adresu ==ADRES_I2C_2BAJTY)//wymiana danych z EEPROMEM 
    {
      test =I2c_byte_wr(i2c.subadres_second);
      if (test ==1) break;//blad
    }
    
    if (i2c_r_w ==1)     //odbior 'ile' bajtow
    {
      I2c_stop();
      I2c_start();
      test =I2c_byte_wr(i2c.adres +1);//adres z bitem r/w=1
      if (test ==1) break;//blad
    }
    for (x=0;x<i2c.ile;x++)
    {
      if (i2c_r_w ==1) //odbior
      {
        if (x !=(i2c.ile-1)) ack =1;
        else ack =0;//dla ostatniego bajtu odbior bez wystawiania ACK
        *(i2c.p_bufor +x) =I2c_byte_rd(ack);
      }
      else             //transmisja
      {
        test =I2c_byte_wr(*(i2c.p_bufor +x));
        if (test ==1) break;//blad
      }
    }
    I2c_stop();
    break;
  }
  if (test ==1) return 0;     //blad EEPROM-u
  else return 1;              //sukces
}
//------------------------------------        
//procedura I2C transmisji bajtu
unsigned char I2c_byte_wr(unsigned char bajt)
//we: bajt -bajt do wyslania
//wy flaga stanu 0 -sukces, 1 -blad transmisji
{
unsigned char bit_count,maska;
unsigned char flaga;

  maska =0x80;
  for (bit_count=8;bit_count>0;bit_count--)//transmisja 8 bitow MSB first
  {
    if ((maska & bajt) ==0) GPIO_EEP_Low(EEP_SDA);//SDA =0
    else 					GPIO_EEP_High(EEP_SDA);//SDA =1
    Delay(1);
    GPIO_EEP_High(EEP_SCL);//SCL =1;
    Delay(5);
    GPIO_EEP_Low(EEP_SCL);//SCL =0;
    Delay(5);
    maska =maska/2;
  }
  GPIO_EEP_High(EEP_SDA);//SDA =in
  Delay(1);
  GPIO_EEP_High(EEP_SCL);//SCL =1;
  Delay(5);
  if (GPIO_EEP_StanLinii(EEP_SDA) ==POZIOM_L) flaga =0;//detekcja ACK
  else flaga =1;
  Delay(5);
  GPIO_EEP_Low(EEP_SCL);//SCL =0;
  					  //SDA =out
  Delay(5);
  GPIO_EEP_Low(EEP_SDA);//SDA =0
  Delay(5);
  return flaga;
}
//---------------------------------
//procedura I2C odbioru bajtu
unsigned char I2c_byte_rd(unsigned char ack_on)
//we: ack_on 1 -wyslij ACK po odbiorze bajtu, 0 -nie wysylaj ACK
//wy: odebrany bajt 
{
unsigned char bit_count,maska,bajt;

  maska =0x80;
  bajt =0;
  GPIO_EEP_High(EEP_SDA);//SDA =in
  Delay(10);
  for (bit_count=8;bit_count>0;bit_count--)//odczyt 8 bitow MSB first
  {
    GPIO_EEP_High(EEP_SCL);//SCL =1;
    Delay(15);
    if (GPIO_EEP_StanLinii(EEP_SDA) !=POZIOM_L) bajt =(bajt | maska);
    GPIO_EEP_Low(EEP_SCL);//SCL =0;
    Delay(15);
    maska =maska/2;
  }
  								 //SDA =out
  if (ack_on ==1)
  {
    GPIO_EEP_Low(EEP_SDA);//SDA =0
    Delay(15);
    GPIO_EEP_High(EEP_SCL);//SCL =1;
    Delay(15);
    GPIO_EEP_Low(EEP_SCL);//SCL =0;
    Delay(10);
  }
  GPIO_EEP_Low(EEP_SDA);//SDA =0
  Delay(15);
  return bajt;
}
//---------------------------------------------------------
//sprawdzenie czy linie SDA i SCL sa w stanie wysokim
unsigned char I2c_cnl(void)
//wy: flaga stanu 0 -sukces, 1 -blaï¿½d
{
char test_SDA, test_SCL;

  I2c_start();
  I2c_stop();
  test_SDA =GPIO_EEP_StanLinii(EEP_SDA);
  test_SCL =GPIO_EEP_StanLinii(EEP_SCL);
  if (test_SDA ==POZIOM_H && test_SCL ==POZIOM_H) return 0;
  else return 1;
/*
  if ((GPIO_EEP_StanLinii(EEP_SDA) ==POZIOM_H) &&
                              (GPIO_EEP_StanLinii(EEP_SCL) ==POZIOM_H)) return 0;
  else return 1;*/
}
//-------------------------
//procedura I2C START
void I2c_start(void)
{
  GPIO_EEP_Low(EEP_SDA);//SDA =0
  Delay(5);
  GPIO_EEP_Low(EEP_SCL);//SCL =0;
  Delay(10);
  return;
}
//------------------------
//procedura I2C STOP
void I2c_stop(void)
{
    GPIO_EEP_High(EEP_SCL);//SCL =1;
    Delay(5);
  	GPIO_EEP_High(EEP_SDA);//SDA =1
	Delay(5);
  	return;
}
//-------------------------------------
//inicjacja portów steruj¹cych liniami SDA SCL EEPROM-u
void GPIO_EEP_Init(void)
{
GPIO_InitTypeDef  GPIO_InitStruct;

	if (LINIA_EEP_PORT[EEP_SDA] ==GPIOA || LINIA_EEP_PORT[EEP_SCL] ==GPIOA)
	{
		__GPIOA_CLK_ENABLE();
	}
	if (LINIA_EEP_PORT[EEP_SDA] ==GPIOB || LINIA_EEP_PORT[EEP_SCL] ==GPIOB)
	{
		__GPIOB_CLK_ENABLE();
	}
	if (LINIA_EEP_PORT[EEP_SDA] ==GPIOC || LINIA_EEP_PORT[EEP_SCL] ==GPIOC)
	{
		__GPIOC_CLK_ENABLE();
	}
	if (LINIA_EEP_PORT[EEP_SDA] ==GPIOD || LINIA_EEP_PORT[EEP_SCL] ==GPIOD)
	{
		__GPIOD_CLK_ENABLE();
	}

	GPIO_InitStruct.Pin = LINIA_EEP_PIN[EEP_SDA];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(LINIA_EEP_PORT[EEP_SDA], &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LINIA_EEP_PIN[EEP_SCL];
	HAL_GPIO_Init(LINIA_EEP_PORT[EEP_SCL], &GPIO_InitStruct);
}
//---------------------------------------
//ustawienie stanu wysokiego na linii EEPROM
void GPIO_EEP_High(Linie_EEP_TypeDef Linia)
{
	HAL_GPIO_WritePin(LINIA_EEP_PORT[Linia], LINIA_EEP_PIN[Linia], GPIO_PIN_SET);
}
//---------------------------------------
//ustawienie stanu niskiego na linii EEPROM
void GPIO_EEP_Low(Linie_EEP_TypeDef Linia)
{
	HAL_GPIO_WritePin(LINIA_EEP_PORT[Linia], LINIA_EEP_PIN[Linia], GPIO_PIN_RESET);
}
//---------------------------------------
//odczyt stanu linii EEPROM
char GPIO_EEP_StanLinii(Linie_EEP_TypeDef Linia)
{
GPIO_PinState stan_linii;

	stan_linii =HAL_GPIO_ReadPin (LINIA_EEP_PORT[Linia], LINIA_EEP_PIN[Linia]);
	if (stan_linii ==GPIO_PIN_RESET) return POZIOM_L;
	else return POZIOM_H;
}
