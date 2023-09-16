#include "debug.h"
#include "i8080.h"

#include "tk80rom.h"

/* Global Variable */
vu8 val;

volatile uint8_t kdstate=0;
volatile uint32_t lastcounter=0;

uint8_t tk80ram[1024];
uint8_t tk80ppi[4];     // i8255 on TK80
uint8_t tk80keypad[4];  // keypad data on TK80

i8080 cpu;

const uint8_t leddata[]={ 0x5c,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x27};

uint8_t memread(void *userdata,uint16_t addr){

    if(addr<0x300) {
        return tk80rom[addr];
    } else if((addr>=0x8000)||(addr<0x8400)) {
        return tk80ram[(addr&0x3ff)];
    }

    return 0;

}

void memwrite(void *userdata,uint16_t addr,uint8_t data){

    if((addr>=0x8000)||(addr<0x8400)) {
        tk80ram[(addr&0x3ff)]=data;
    }
}


uint8_t ioread(void *userdata,uint8_t addr){


    switch(addr&3) {
    case 0:

        if((tk80ppi[2]&0x10)==0) { return tk80keypad[0]; }
        if((tk80ppi[2]&0x20)==0) { return tk80keypad[1]; }
        if((tk80ppi[2]&0x40)==0) { return tk80keypad[2]; }

        return 0xff;

    default:
        return 0xff;

    }

    return 0xff;

}

void iowrite(void *userdata,uint8_t addr,uint8_t data){

    uint8_t bit;

    tk80ppi[addr&3]=data;
    if((addr&3)==2) {   // Port C
        if((data&2)==0) {  // Buzzer (bit 1)
            GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
        } else {
            GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
        }
    }
    if((addr&3)==3) {   // Port C bit operation

        if((data&0x80)==0) {
            bit=1<<((data>>1)&7);
            if((data&0)==0) {
                tk80ppi[2]&= ~bit;
                if(bit==2) {   // Buzzer (bit 1)
                    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
                }
            } else {
                tk80ppi[2]|=bit;
                if(bit==2) {  // Buzzer (bit 1)
                    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
                }
            }
        }

    }

}


static inline void gpioc_set(GPIOMode_TypeDef gpio_mode) {

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = gpio_mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);

}

static inline void gpiod_set(uint16_t gpio_pin, BitAction bit_state  ) {

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    // set all pins to High-Z

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init( GPIOD, &GPIO_InitStructure);
    GPIOD->CFGLR=0x44444444;

    GPIO_InitStructure.GPIO_Pin = gpio_pin ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOD, gpio_pin, bit_state);

}

void timer_init() {

    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1 , ENABLE);

    // Initalize TIM1

    TIM_TimeBaseInitStructure.TIM_Period = 3050;          // 15.75kHz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM1, ENABLE);
    TIM_Cmd( TIM1, ENABLE);

    // NVIC

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);

}

void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM1_CC_IRQHandler(void) {

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    TIM_ClearFlag(TIM1, TIM_FLAG_CC4);

    if (kdstate < 4) {        // for Common Cathode LED
//        gpioc_set(GPIO_Mode_Out_PP);
        GPIOC->CFGLR=0x33333333;
        gpiod_set(1 << (kdstate & 3), Bit_RESET); // Low

        if((tk80ppi[2]&0x80)!=0) {
         GPIOC->OUTDR=tk80ram[0x3f8+kdstate];
        } else {
            GPIOC->OUTDR=0;
        }

    } else if (kdstate < 8) {  // for Common Anode LED
//        gpioc_set(GPIO_Mode_Out_PP);
        GPIOC->CFGLR=0x33333333;

        gpiod_set(1 << (kdstate & 3), Bit_SET); // High

        if((tk80ppi[2]&0x80)!=0) {
        GPIOC->OUTDR=~tk80ram[0x3f8+kdstate];
        } else {
            GPIOC->OUTDR=0xff;
        }

    } else {
//        gpioc_set(GPIO_Mode_IPU);
        GPIOC->CFGLR=0x88888888;
        GPIOC->OUTDR=0xff;

//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_Init( GPIOD, &GPIO_InitStructure);
        GPIOD->CFGLR=0x44444444;

        GPIO_InitStructure.GPIO_Pin = 1<<(kdstate-4);  // PD4-7
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init( GPIOD, &GPIO_InitStructure);

        GPIO_WriteBit(GPIOD, 1<<(kdstate-4), Bit_RESET);

        tk80keypad[kdstate-8]=GPIOC->INDR;

    }
    kdstate++;
    if (kdstate > 12)
        kdstate = 0;

//    cycle_count=cpu.cyc;
//
//    while(cpu.cyc<(cycle_count + 1)) {    //  63.5us / 2048kHz  = 130 cycles * wait (4/5)
//
//    }

}


void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{

    uint32_t stepflag=0;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    // set all pins to High-Z

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure);

    GPIOD->BSHR=0x00f0;  // set HIGH PD4-7

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2  ;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init( GPIOA, &GPIO_InitStructure);


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    SystemCoreClockUpdate();
      Delay_Init();

//      USART_Printf_Init(115200);
//      printf("SystemClk:%d\r\n", SystemCoreClock);
//      printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
//
//      USARTx_CFG();


#ifdef DEBUG_TK80

      // DEBUG

//      for(int i=0;i<TK80EXAMPLE1_BYTES;i++) {
//          tk80ram[0x200+i]=tk80example1[i];
//      }

      for(int i=0;i<TK80EXAMPLE4_BYTES;i++) {
          tk80ram[0x200+i]=tk80example4[i];
      }

#endif

    i8080_init(&cpu);

    cpu.read_byte = memread;
    cpu.write_byte = memwrite;
    cpu.port_in = ioread;
    cpu.port_out = iowrite;

    // disable SWIO

    GPIO_PinRemapConfig(GPIO_Remap_SDI_Disable, ENABLE);

    timer_init();


    while(1)
    {

        if(((tk80keypad[3]&2)==0)&&(cpu.iff==1)) { // STEP mode
            stepflag++;
            if(stepflag>2) {       // Interrupt (RST 7) next op fetch cycle
                i8080_interrupt(&cpu, 0xff);
            }
        } else {
            stepflag=0;
        }
        if((tk80keypad[3]&1)==0) { // RESET
            cpu.pc=0;
        }

        i8080_step(&cpu);

              if(cpu.pc==0xd5) { // STORE DATA

                  uint32_t data;

                  FLASH_Unlock_Fast();

                  // Erase flash from 0x8003c00 to 0x8003fff
                  for(int i=0;i<16;i++) {
                      FLASH_ErasePage_Fast(0x08003c00+i*64);
                  }

                  // Program flash
                  for(int i=0;i<16;i++) {
                      FLASH_BufReset();
                      for(int j=0;j<64;j+=4) {
                          data=(tk80ram[i*64+j+3]<<24)+(tk80ram[i*64+j+2]<<16)+(tk80ram[i*64+j+1]<<8)+tk80ram[i*64+j];
                          FLASH_BufLoad(0x08003c00+i*64+j, data);
                      }
                      FLASH_ProgramPage_Fast(0x08003c00+i*64);
                  }
                  FLASH_Lock_Fast();

                  cpu.pc=0;
              }
              if(cpu.pc==0x107) { // LOAD DATA
                  for(int i=0;i<1024;i++) {
                      tk80ram[i]=*(uint8_t *)(0x08003c00+i);
                  }
                  cpu.pc=0;
              }

    }
}
