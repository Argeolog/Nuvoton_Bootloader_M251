
	
	
	/***************************************************************************//**
 * @file     isp_user.c
 * @brief    ISP Command source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "string.h"
#include "isp_user.h"
#include "Degiskenler.h"
__attribute__((aligned(4))) uint8_t GidenData[64];
__attribute__((aligned(4))) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];

uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;


uint32_t StartAddress=0;
uint8_t FmcYazmaAktif=0;
uint8_t FlashData[16];
uint8_t Forix=0;
uint32_t FlashAdres=0;
uint8_t islemKodu=0;
uint8_t HexDosyaAdiEslesti=0;
unsigned char Okunanveri;
unsigned char CihazAdi[10];
 uint8_t CihazAyarBuffer[512];


int GelenData(unsigned char *buffer, uint8_t len)
{
	islemKodu=buffer[0];
  GidenData[0]=islemKodu;
	GidenData[1]=0;
	
if (islemKodu==CihazAdiEslestir){

	 uint8_t CihazAdiHatali=0;
	  HexDosyaAdiEslesti=0;
	 for (Forix=0;Forix<=7;Forix++){ // Cihaz Ismimiz Için 8 Bytelik Yer Ayiriyoruz [MyKey] Gibi.
		 Okunanveri=Flash_Read_8(CihazAdiAdres+Forix); // 0xC004 Adresinden byte Byte Veri okuyoruz.
     if (Okunanveri!= buffer[Forix+1]){ // Okudugumuz Veri ile Gelen Verinin Birbirine Esit olup olmadigini sorguluyouruz.Buffer[Forix+1] +1 yapmamizin sebebi ilk Veri Islem Verisi.
			  CihazAdiHatali=1;
			  HexDosyaAdiEslesti=0;
			  break;
		 }			 
 		 if (Okunanveri==']'){ // Flash Hafizadan okugumuz veri ] ise Artik kodun sonu gelmis demektir.
			 HexDosyaAdiEslesti=1; // Son Koda kadar okuduk ve bir kod kirilmadigi için islem onaylanmis olur.
			 break;
		 }
		}
	 GidenData[1]=HexDosyaAdiEslesti; // Cihaz Adinin dogru oldugu bilgisini gönderiyoruz.
	}
	
	
	
	
	
	if (islemKodu==CihazAdiOku){
	
	  HexDosyaAdiEslesti=0;
	 for (Forix=0;Forix<=7;Forix++){ // Cihaz Ismimiz Için 8 Bytelik Yer Ayiriyoruz [MyKey] Gibi.
			Okunanveri=Flash_Read_8(CihazAdiAdres+Forix); // 0xC004 Adresinden byte Byte Veri okuyoruz.
     GidenData[2+Forix]=Okunanveri;
		}
	 GidenData[1]=1; // Cihaz Adinin dogru oldugu bilgisini gönderiyoruz.
	
	}
	
			
	
	if (islemKodu==CihazAdiYaz)
		{
	  ReadData(CihazAyarPageStart, CihazAyarPageEnd, (uint32_t *)CihazAyarBuffer);
		Flash_Sil(CihazAyarPageStart,1);	
    for( Forix=0;Forix<=7;Forix++){
		   CihazAyarBuffer[Forix+4]=buffer[Forix+1];
	  }
   
		WriteData(CihazAyarPageStart, CihazAyarPageEnd, (uint32_t *)CihazAyarBuffer);
	  HexDosyaAdiEslesti=0;
		GidenData[1]=1; // Cihaz Adi Yazma Basarili.
		HexDosyaAdiEslesti=0;
		  }
	

  if (islemKodu==FlashSil)
		  {
	      Flash_Sil(FMC_APROM_BASE,ProgramSayfaSayisi);	   
    		GidenData[1]=1; // Flash Silme Basarili.
	    }
	
	 
	   if (islemKodu==CihazAyarFlashSil)
			 {
	      Flash_Sil(CihazAyarPageStart,1);	   
    		GidenData[1]=1; // Flash Silme Basarili.   
		   }
	
	
	
 if (HexDosyaAdiEslesti==1){

	if (islemKodu==FlashYaz){
						
	for( Forix=0;Forix<=15;Forix++){
		   FlashData[Forix]=buffer[Forix+1];
	  }
			
	if (FmcYazmaAktif==0)
		 {
	   FmcYazmaAktif=1;	
	 	 Flash_Sil(FMC_APROM_BASE,ProgramSayfaSayisi);	
     }
		 
	  WriteData(FlashAdres, FlashAdres+16, (uint32_t *)FlashData);
    FlashAdres+=16;
	  GidenData[1]=1; // Flash Veri Yazildi.
	  	
  }
	
	else if (islemKodu==YazmaTamamlandi)
	 {	
		ReadData(CihazAyarPageStart, CihazAyarPageEnd, (uint32_t *)CihazAyarBuffer); // Önce Flasdaki Veriyi Okuyup Bir Liste Aliyoruz.
		Flash_Sil(CihazAyarPageStart,1);	// Cihazin Ayar Sayfasini siliyoruz.
    CihazAyarBuffer[0]=0; // Boot Adresinin tutuldugu alani 0 Yapiyoruz. Bootdan çikmasi için.
		WriteData(CihazAyarPageStart, CihazAyarPageEnd, (uint32_t *)CihazAyarBuffer); //Sonra Bu ayarlari Tekrar Yaziyoruz..
		GidenData[1]=1; // Boot Yazma Tamamlandi.
		EP2_Handler();	
		 uint32_t ix;
		 CLK_SysTickLongDelay(10000);
		  outpw(&SYS->RSTSTS, 3);//clear bit
		 ix = (FMC->ISPCTL & 0xFFFFFFFC);
		  outpw(&FMC->ISPCTL, ix);
		 NVIC_SystemReset(); 
		 return 0;
		 
	 }
		 
 }
	
 

 
 
		EP2_Handler();	
	  return 0;
   
}

