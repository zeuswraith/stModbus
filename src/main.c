#include "main.h"
#include "modbus.h"
#include "dev/sample.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f103xb.h"
Modbus_Conf_t *pconf;

volatile char data1[8];
uint32_t device_reg_4xxxx[12];
uint16_t output;
uint16_t input;
uint16_t adc[3];
uint16_t dac[16];
mbus_t mb;
uint8_t i=0;


mbus_t mbus_somedevice_open(Modbus_Conf_t *pconf)
{
	 mbus_t mb;
	 pconf->devaddr=0x01;

	//Define read callback function
    pconf->read = mbus_somedev_read;

	//Define write callback function
    pconf->write = mbus_somedev_write;

    //Open modbus context
	mb = mbus_open(pconf);

    if (mb < 0 ) return (mbus_t)MBUS_ERROR;

	//We can map some function on modbus commands
    //if ( mbus_connect(mb, mbus_somedevice_read_3xxxx, MBUS_FUNC_READ_INPUT_REGS ) ) return (mbus_t)MBUS_ERROR;

    //if ( mbus_connect(mb, mbus_somedevice_read_4xxxx, MBUS_FUNC_READ_REGS ) ) return (mbus_t)MBUS_ERROR;

	//Set default values for registers as you need

    device_reg_4xxxx[0] = 0x322A;             //40001
    device_reg_4xxxx[1] = pconf->devaddr;     //40002 Device addr
    device_reg_4xxxx[2] = 0x7;                //40003

    device_reg_4xxxx[3] = 0x0;                //40004
    device_reg_4xxxx[4] = 0x0;                //

    device_reg_4xxxx[5] = 0x0;                //40006
    device_reg_4xxxx[6] = 0x0;                //

    device_reg_4xxxx[7] = 0x0;                //40008
    device_reg_4xxxx[8] = 0x0;                //

    device_reg_4xxxx[11] = 0x0;                //Up time // 40012


    return mb;
}



void clkcfg()
{

	 RCC->APB1ENR |=RCC_APB1ENR_USART2EN;
	 RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;

}
void gpiocfg()
{
	GPIOA->CRL &= ~(15<<8);
	 GPIOA->CRL |=(11<<8);
	 GPIOA->CRL &= ~(15<<12);
	 GPIOA->CRL |=(8<<12);
	}


void uartcfg()
{  uint16_t uartdiv =8000000 / 9600;
	clkcfg();
	gpiocfg();
	USART2->CR1 = 0x00000000;
		USART2->CR2 = 0x00000000;
		USART2->CR3 = 0x00000000;



		USART2->CR1 |=USART_CR1_IDLEIE;
	//USART2->CR1 |=USART_CR1_RXNEIE;
	USART2->CR1 |= USART_CR1_UE;
	 USART2->CR1 &= ~USART_CR1_M;
	 USART2->CR1 |=USART_CR1_TE ;
	 USART2->CR1 |=USART_CR1_RE ;

	 USART2->BRR |=(((uartdiv / 16)<< USART_BRR_DIV_Mantissa_Pos)|
			        ((uartdiv % 16)<< USART_BRR_DIV_Fraction_Pos ));

NVIC_SetPriority(USART2_IRQn,0);
NVIC_EnableIRQ(USART2_IRQn);
RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//reception
	DMA1_Channel6->CCR = 0x00000000;
	DMA1_Channel6->CCR |= DMA_CCR_PL_1 ;
DMA1_Channel6->CCR |= DMA_CCR_PL_0 ;
DMA1_Channel6->CMAR = (uint32_t)data1 ;
    	DMA1_Channel6->CPAR = (uint32_t) &USART2->DR;
    	DMA1_Channel6->CCR &= ~ DMA_CCR_DIR;
		DMA1_Channel6->CCR &= ~ DMA_CCR_MSIZE_1;
		DMA1_Channel6->CCR &= ~ DMA_CCR_MSIZE_0;

		DMA1_Channel6->CCR &= ~  DMA_CCR_PSIZE_1;
		DMA1_Channel6->CCR &= ~  DMA_CCR_PSIZE_0;

		DMA1_Channel6->CCR &= ~ DMA_CCR_PINC;
		DMA1_Channel6->CCR |= DMA_CCR_MINC;
		DMA1_Channel6->CCR |= DMA_CCR_CIRC ;

		DMA1_Channel6->CNDTR = 8;

		DMA1_Channel6->CCR |= DMA_CCR_EN;


		//transmission
	//	DMA1_Channel7->CCR = 0x00000000;
//		DMA1_Channel7->CCR |= DMA_CCR_PL_1 ;
	//	DMA1_Channel7->CCR |= DMA_CCR_PL_0 ;
	//	DMA1_Channel7->CMAR = (uint32_t)data ;
	   // 	DMA1_Channel7->CPAR = (uint32_t) &USART2->DR;
	    //	DMA1_Channel7->CCR |= DMA_CCR_DIR;
		//	DMA1_Channel7->CCR &= ~ DMA_CCR_MSIZE_1;
		//	DMA1_Channel7->CCR &= ~ DMA_CCR_MSIZE_0;

		//	DMA1_Channel7->CCR &= ~  DMA_CCR_PSIZE_1;
		//	DMA1_Channel7->CCR &= ~  DMA_CCR_PSIZE_0;

		//	DMA1_Channel7->CCR &= ~ DMA_CCR_PINC;
		//	DMA1_Channel7->CCR |= DMA_CCR_MINC;
		//	DMA1_Channel7->CCR |= DMA_CCR_CIRC ;
		//	DMA1_Channel7->CCR |= DMA_CCR_TCIE;
		//	DMA1_Channel7->CNDTR = 9;

			//DMA1_Channel7->CCR |= DMA_CCR_EN;
	//	NVIC_SetPriority(DMA1_Channel7_IRQn,0);
		//NVIC_EnableIRQ(DMA1_Channel7_IRQn);
		USART2->CR3 |= USART_CR3_DMAR;
		USART2->CR3 |= USART_CR3_DMAT;


}


	//We can map some function on modbus commands
    //if ( mbus_connect(mb, mbus_somedevice_read_3xxxx, MBUS_FUNC_READ_INPUT_REGS ) ) return (mbus_t)MBUS_ERROR;

    //if ( mbus_connect(mb, mbus_somedevice_read_4xxxx, MBUS_FUNC_READ_REGS ) ) return (mbus_t)MBUS_ERROR;

	//Set default values for registers as you need



uint16_t mbus_somedev_read(uint32_t la)
{
	switch(la){
				case 40001:
					return data1[0];
				case 40002:
					return data1[1];
				case 40003:
									return data1[2];
				case 40004:
									return data1[3];
				case 40005:
									return data1[4];
				case 40006:
									return data1[5];
				case 40007:
									return data1[6];
}return la;}

uint16_t mbus_somedev_write(uint32_t la, uint16_t value)
{

		if ( la > 40000 && la <= 40016 ){
			uint8_t ch  = la - 40001;
			dac[ch] = value;
			//spi_dac_set(dac[ch]&0xFF,ch);
		}
		if (la == 40017 ){
			//ion_adc_writereg(0x1, 1, value&0xFF);
		}
		if (la == 40018 ){
			//ion_mux_select(value&0xF);
			//ion_mux_output(value&0x10);
		}



    return value;
}
//void DMA1_Channel7_IRQHandler()
//{if (DMA1->ISR & DMA_ISR_TCIF7){
	//DMA1->IFCR &= ~DMA_IFCR_CTCIF7;
//	if(DMA1_Channel7->CNDTR == 7)
//	{DMA1_Channel7->CNDTR = 0;}}





//}
void USART2_IRQHandler(){
if ((USART2->SR & USART_SR_IDLE)){

if (data1[7]!=0){DMA1_Channel6->CNDTR = 0;}
for (i=0;i<8;i++) {
mbus_poll(mb, data1[i] );}
USART1->SR &= ~USART_SR_IDLE;

	}
	//if ((USART2->SR & USART_SR_RXNE))
	//{if (i>8){i=0;}
//USART1->SR &= ~USART_SR_RXNE;
//data1[i]=USART2->DR;
//i++;}
	//if ((USART2->SR & USART_SR_TXE))
//	{if (i>16){i=0;}
	//USART1->SR &= ~USART_SR_TXE;
	//	USART2->DR=device_reg_4xxxx[i];
	//i++;}
	}



int main(void)
{
	mbus_somedevice_open(pconf);
	uartcfg();




 while(1)
 {


	}
}

