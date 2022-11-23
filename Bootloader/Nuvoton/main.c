/***************************************************************************//**
 * @file     main.c
 * @version  V3.03
 * @brief    Demonstrate how to update chip flash data through USB HID interface
             between chip USB device and PC.
             Nuvoton NuMicro ISP Programming Tool is also required in this
             sample code to connect with chip USB device and assign update file
             of Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 
 Hasan Atmaca
 23.11.2022
 argelojik@gmail.com
 
 
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "hid_transfer.h"
#include "Degiskenler.h"
#define TRIM_INIT           (SYS_BASE + 0x118)
#define PLL_CLOCK           48000000

	
	
uint8_t  Flash_Read_8(uint32_t Adres){
	  FMC_EnableAPUpdate();		
	  FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = Adres;
    FMC->ISPDAT = 0u;
    FMC->ISPTRG = 0x1u;
	  
#if ISBEN
    __ISB();
#endif
     ;

	
 uint32_t OkunanVeri= FMC->ISPDAT;
 uint8_t AdresSatiri=0;
 AdresSatiri=Adres & 0x000000ff;
	

	if ((AdresSatiri==0x00) || (AdresSatiri==0x04)|| (AdresSatiri==0x08) || (AdresSatiri==0x0C)) {
		return (OkunanVeri&0x000000ff);
	}
	
	
	if ((AdresSatiri==0x01) || (AdresSatiri==0x05) || (AdresSatiri==0x09) || (AdresSatiri==0x0D)) {
		return (OkunanVeri & 0x0000ff00)>> 8;
	}
	
	if ((AdresSatiri==0x02) || (AdresSatiri==0x06) || (AdresSatiri==0x0A) || (AdresSatiri==0x0E)) {
 return (OkunanVeri & 0x00ff0000)>> 16;
	}
	
	if ((AdresSatiri==0x03) || (AdresSatiri==0x07) || (AdresSatiri==0x0B) || (AdresSatiri==0x0F)) {
	
		return (OkunanVeri & 0xff000000) >> 24;
		
	}
	
	return 255;
}










uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for external XTAL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */

    SystemCoreClock = __HIRC;             // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk | CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_EXSTCKEN_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA.12 ~ PA.14 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
}


 
unsigned int YardimciBootMode=0;
void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    uint32_t u32TrimInit;
    SYS_UnlockReg();
    SYS_Init();
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
  
	
	uint8_t	BootMode=Flash_Read_8(BootModeAdres);
	
	
    while (BootMode!=0)
    {
     
   
			USBD_Open(&gsInfo, HID_ClassRequest, NULL);
      
        HID_Init();
        USBD_Start();
       u32TrimInit = M32(TRIM_INIT);
      
        USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

        /* Using polling mode and Removed Interrupt Table to reduce code size for M251 */
        while (BootMode!=0)
        {
            /* Start USB trim function if it is not enabled. */
            if ((SYS->HIRCTRIMCTL & SYS_HIRCTRIMCTL_FREQSEL_Msk) != 0x1)
            {
                /* Start USB trim only when USB signal arrived */
                if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
                {
                    /* Clear SOF */
                    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                    /*
                        USB clock trim function:
                        HIRC Trimming with boundary function enhances robustility
                        and keeps HIRC in right frequency while receiving unstable USB signal
                    */
                    SYS->HIRCTRIMCTL = (0x1 << SYS_HIRCTRIMCTL_REFCKSEL_Pos)
                                       | (0x1 << SYS_HIRCTRIMCTL_FREQSEL_Pos)
                                       | (0x0 << SYS_HIRCTRIMCTL_LOOPSEL_Pos)
                                       | (0x1 << SYS_HIRCTRIMCTL_BOUNDEN_Pos)
                                       | (10  << SYS_HIRCTRIMCTL_BOUNDARY_Pos);
                }
            }

            /* Disable USB Trim when any error found */
            if (SYS->HIRCTRIMSTS & (SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk))
            {
                /* Init TRIM */
                M32(TRIM_INIT) = u32TrimInit;

                /* Disable USB clock trim function */
                SYS->HIRCTRIMCTL = 0;

                /* Clear trim error flags */
                SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk;

                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
            }

            // polling USBD interrupt flag
            USBD_IRQHandler();

            if (bUsbDataReady == TRUE)
            {
                GelenData((uint8_t *)usb_rcvbuf, 64);
                bUsbDataReady = FALSE;
							  
            }
        }

        goto _APROM;
    }

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

_APROM:
    outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}
