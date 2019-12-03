#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "i2c_lcd.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stdio.h"

void delay(uint32_t t);

char key1;
char key2;
char pswd[10];

int centaine;
int dizaine;
int unite;

int PWM;
int ncount=0;

int j=0;
int K=0;
int a;
const uint8_t mes[] = "CONFIG_SYSTEM  ";

char txt[10];

char CODE[4]="ABCD";

/**********************************************************************************
    //System clock TIMER SYSCLK=168MHz
 **********************************************************************************/

void SystemInit_1() /// CLK_TIMER=84MHz
{

  RCC->CFGR |= 0x00009400;        // AHB_presc=1  APB1_presc=4
  RCC->CR |= 0x00010000;          // HSE_ON
  while (!(RCC->CR & 0x00020000));// wait until HSE READY

  RCC->PLLCFGR = 0x07402A04;      // PLL  M=4, N=168, P=2  Q=7 yapalim   168 Mhz
  RCC->CR |= 0x01000000;          // PLL_ON
  while(!(RCC->CR & 0x02000000)); // wait Pll_ready

  FLASH->ACR = 0x00000605;        // Flash ROM icin 5 Wait state secelim ve ART yi aktif edelim (Rehber Sayfa 55)
  RCC->CFGR |= 0x00000002;        // Sistem Clk u PLL uzerinden besleyelim
  while ((RCC->CFGR & 0x0000000F) != 0x0000000A); // Besleninceye kadar bekle

}

/**********************************************************************************
    //Preparation de l'interface TIMER3 permettant de générer un signal PWM : PIN :PA6(CH1) PA7(CH2) PB0(CH3) PB1(CH4)
 **********************************************************************************/
void Config_PWM()  /// APB1_TIMER_3_CLOCK=84MHz
{

   RCC->APB1ENR |= 0x2; //clock enable for TIM3
   RCC->AHB1ENR |= 0x3; //clock enable GPIOA GPIOB
   GPIOA->MODER |= 0xA000; // PA6 and PA7  OUTPUT
   GPIOB->MODER |= 0x000A; // PB0 and PB1 OUTPUT
   GPIOA->PUPDR=0x22222222;
   GPIOB->PUPDR=0x22222222;

  TIM3->CCMR1 |= 0x6060; //PWM mode 1 on TIM3 Channel 1 & TIM3 Channel 2 => PA6  PA7
  TIM3->CCMR2 |= 0x6060; //PWM mode 1 on TIM3 Channel 3 & TIM3 Channel 4 => PB0 PB1

   TIM3->PSC = 1000;  //CLK_TIMER3= 84.000.000/1000=84.000Hz
   TIM3->ARR = 320;   //Periode_PWM= (1/84000)x320=0.00381s  ==> Freq_PWM= 1/0.00381=262Hz
   TIM3->CCR1 = 0;
   TIM3->CCR2 = 0;
   TIM3->CCR3 = 0;
   TIM3->CCR4 = 0;
   GPIOA->AFR[0]|= 0x22000000; //set GPIOA to AF2
   GPIOB->AFR[0] = 0x00000022; //set GPIOA to AF2

   TIM3->BDTR|=0xc000;
   TIM3->CCER|=0x1111; //Enable Capture/Compare 1 output  &&   Enable Capture/Compare 1 output
  // TIM3->EGR|=0x06;  // Capture/compare 1 generation && Capture/compare 2 generation
   TIM3->CR1 |= 0x1; //enable timer 3
}


void configuration()
{
	RCC->AHB1ENR=0x00000008;
	GPIOD->MODER=0x55550000; //PD15 ->PD12 OUTPUT
	GPIOD->PUPDR=0x55555555;
}

void gpioInit(void)
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //PB6 SCL   PB7  SDA
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &gpio);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
}

void i2cInit(void)
{
	I2C_InitTypeDef i2c;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	i2c.I2C_ClockSpeed = 50000;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &i2c);
	I2C_Cmd(I2C1, ENABLE);
}

void Init_I2C()
{

//configure the AF
  RCC->AHB1ENR |= 1<<1;       //Clock for  GPIOB
  RCC->APB1ENR |= 1 <<21; //clock to I2C1

  GPIOB->AFR[0] |= (4<< 24);  //enable SCK to PB6
  GPIOB->AFR[0] |= (4<< 28);  //enable SDA to PB7

  GPIOB->MODER &= ~(3 << 12); //clear bits 12 & 13 (PB6)
  GPIOB->MODER |= 2 << 12; //MODER6[1:0] = 10 bin
  GPIOB->MODER &= ~(3 << 14); //clear bits 14 & 15 (PB7)
  GPIOB->MODER |= 2 << 14; //MODER9[1:0] = 10 bin

  GPIOB->OTYPER |= 1 <<6; //PB6 open drain
  GPIOB->OTYPER |= 1<<7; //PB7 open drain
  GPIOB->PUPDR &= ~(3 << 12);//clear bits 12 & 13 (PB6)
  GPIOB->PUPDR &= ~(3 << 14);//clear bits 14 & 15 (PB7)

  //Configure the I2C
  	I2C1->CR2     = 0x0010; //16MHz must inferieur a 42MHZ
  	I2C1->CCR     = 0x0050; //100kHz Bit rate
  	I2C1->TRISE   = 0x0011; //1000 ns / 62.5 ns = 16 + 1
  	I2C1->CR1    = 0x0001;      //enable peripheral
}
void delay(uint32_t t)
{
 	uint32_t i = 0;
 	for (; i < t; i++);
 }


void RECEIVE_PWM()
{
	 key1=Getkey();
	    	 if(key1!=239)
	    	 {

	    	    	   sprintf(txt,"%c",key1);
	    	    	   lcd_Goto(1,11+K);
	    	    	   lcd_PrintC(txt);
	    	    	   K++;
	    	    	   if(K==1)
	    	    	   {
	    	    	     centaine=key1-48;
	    	    	     ncount++;
	    	    	   }
	    	    	   if(K==2)
	    	    	   {
	    	    	   	  dizaine=key1-48;
	    	    	   	ncount++;
	    	    	   	}
	    	    	   if(K==3)
	    	    	   	{
	    	    	   	  unite=key1-48;
	    	    	   	  ncount++;

	    	    	   	  PWM=(100*centaine)+(10*dizaine)+unite;
	    	    	   	  K=0;
	    	    	   	  TIM3->CCR1=PWM;
	    	    	   	delay(8000000);
	    	    	   	lcd_Goto(1,5);
	    	    	      lcd_PrintC("PWM SAVED");
	    	    	      delay(8000000);

	    	    	   	}
	    	    	   delay(8000000);



	    	    }
}


 int main(void)
 {
	 SystemInit_1();  // initialisation  SYSTEM CLOCK
	 Config_PWM();
	 Init_I2C();


	configuration();
 	lcd_Init();

 	lcd_Goto(1, 2);
 	lcd_PrintC(mes);
 	lcd_Goto(2, 0);
 	lcd_PrintC("MACHINE A LAVER");
 	delay(80000000);
 	lcd_Command(0x1);

     while(1)
     {

    	 lcd_Goto(1, 0);
    	 lcd_PrintC("PSWD:");

     	 key1=Getkey();
    	 if(key1!=239)
    	 {

    	    	   sprintf(txt,"%c",key1);
    	    	   lcd_Goto(1,6+j);
    	    	   lcd_PrintC(txt);
    	    	   j++;
    	    	   pswd[j-1]=key1;
    	    	   delay(8000000);
    	    	   if(j==4)
    	    	   {
    	    		   a=strcmp(CODE,pswd);

    			       if(a==1)
    			       {
    			             GPIOD->ODR=0x8000;
    			             delay(10000000);
    			             GPIOD->ODR=0x0000;
    			             lcd_Command(0x1);
    			             lcd_Goto(1,6);
    			             lcd_PrintC("CODE OK");

    			             delay(20000000);
    			             lcd_Command(0x1);

    			             lcd_Goto(1,0);
    			             lcd_PrintC("PWM VALUE:");
    			             while(ncount<3)
    			              {
    			                 RECEIVE_PWM();
    			              }
    			              ncount=0;
    			             delay(20000000);
    			             lcd_Command(0x1);
                             lcd_Goto(1,0);
    			             lcd_PrintC("PWM VALUE:");

    			       }
    			       else
    			       {
    				         GPIOD->ODR=0x0000;
    				         lcd_Command(0x1);
    				         lcd_Goto(1,6);
    				         lcd_PrintC("CODE NOK");

    				         delay(30000000);
    			       }

    	    		   j=0;
    	    		   lcd_Command(0x1);
    	    		   lcd_Goto(1, 0);
    	    	       lcd_PrintC("PSWD:");




    	    	   }

    	    }

    	 }





 }

