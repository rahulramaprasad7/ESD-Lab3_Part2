#include "msp.h"
#include <stdio.h>


int check =0;
volatile uint32_t temp;
uint16_t x;
int32_t adcRefTempCal_1_2v_30;
int32_t adcRefTempCal_1_2v_85;

uint16_t pwmLevel = 4;


#define PWMCOUNT 6553

volatile long degree;
volatile float IntDegF;
volatile float IntDegC;

void temp_init()
{
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);
    REF_A->CTL0 |= REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON;                    //Enable REFON bit

    REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;

    ADC14->CTL0 = ADC14_CTL0_SHT0_6 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 |= ADC14_CTL1_TCMAP;  //Enable Temperature sensor and use unsigned adc

    ADC14->MCTL[0] |= ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_22;            //Set input

    ADC14->IER0 |= 0x0001;
    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

void push_init()
{
    P1->DIR |= ~(uint8_t) BIT1 | ~(uint8_t) BIT4;
    P1->OUT |= BIT1 | BIT4;
    P1->REN |= BIT1 |BIT4;                          // Enable pull-up resistor (P1.1 output high)
    P1->SEL0 = 0;
    P1->SEL1 = 0;
    P1->IES |= BIT1 | BIT4;                         // Interrupt on high-to-low transition
    P1->IFG = 0;                                    // Clear all P1 interrupt flags
    P1->IE |= BIT1 | BIT4;                          // Enable interrupt for P1.
}

void uart_init()
{

        P1->SEL0 |= BIT2 | BIT3;

        EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;
        EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;
        EUSCI_A0->BRW = 6;
        EUSCI_A0->MCTLW = (0xAA << EUSCI_A_MCTLW_BRS_OFS);

        EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
        EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;
        EUSCI_A0->IE |= EUSCI_A_IE_RXIE;
}

void timer_init()
{
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_6 | TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_OUTMOD_6 | TIMER_A_CCTLN_CCIE; // TACCR1 interrupt enabled

    TIMER_A0->CCR[0] = 65531;
    TIMER_A0->CCR[1] = 4 * PWMCOUNT;

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__CONTINUOUS; // SMCLK, continuous mode

}
void main(void)

{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;
    adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;
    temp_init();
    timer_init();
    push_init();
    uart_init();

    P1->DIR |= BIT0;
    P1->OUT |= BIT0;


    P2->DIR |= 0xFF; P2->OUT = 0;
    P3->DIR |= 0xFF; P3->OUT = 0;
    P4->DIR |= 0xFF; P4->OUT = 0;
    P5->DIR |= 0xFF; P5->OUT = 0;
    P6->DIR |= 0xFF; P6->OUT = 0;
    P7->DIR |= 0xFF; P7->OUT = 0;
    P8->DIR |= 0xFF; P8->OUT = 0;
    P9->DIR |= 0xFF; P9->OUT = 0;
    P10->DIR |= 0xFF; P10->OUT = 0;

     __enable_irq();

    NVIC->ISER[1] |= 1 << ((PORT1_IRQn) & 31);
    NVIC->ISER[0] |= (1 << (EUSCIA0_IRQn & 31)) | (1 << (TA0_0_IRQn & 31)) | (1 << (TA0_N_IRQn & 31)) | (1 << ((ADC14_IRQn) & 31)) ;

    while(1)
    {
        ADC14->CTL0 |= ADC14_CTL0_SC;
        IntDegC = (((float) degree - adcRefTempCal_1_2v_30) * (85 - 30)) / (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;
        IntDegF = ((9 * IntDegC) / 5) + 32;
        //printf("%f ", IntDegC);

//        if (x == 'p')
//            printf("%d", pwmLevel);
//        printf("\n");

        if (check == 1)
        {
            if(pwmLevel != 0)
            {
                if (TIMER_A0->CCR[1] > 0)
                {
                    pwmLevel--;
                    TIMER_A0->CCR[1] = pwmLevel * PWMCOUNT;
                }
                check = 0;
                printf("Duty cycle is %u% ", (pwmLevel * 10));
                    printf("\n");
            }
        }
        if (check == 2)
        {
            if(pwmLevel != 10)
            {
                if (TIMER_A0->CCR[1] <65530)
                {
                    pwmLevel++;
                    TIMER_A0->CCR[1] = pwmLevel * PWMCOUNT;
                }
                check = 0;
                printf("Duty cycle is %u% ", (pwmLevel * 10));
                    printf("\n");
            }
        }
    }

}
/* Port1 ISR */
void PORT1_IRQHandler(void)
{
    volatile uint32_t i;

    if(P1->IFG & BIT1)
    {
        // Delay for P1.1 switch debounce
        for(i = 0; i < 5000; i++)
            P1->IFG &= ~BIT1;
        check = 1;
    }



    else if(P1->IFG & BIT4)
    {
        // Delay for P1.4 switch debounce
       for(i = 0; i < 5000; i++)
           P1->IFG &= ~BIT4;
       check = 2;
    }

}

void TA0_0_IRQHandler(void)
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT |= BIT0;
}

void TA0_N_IRQHandler(void)
{
    TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT &= ~BIT0;
}

void ADC14_IRQHandler(void)
{
    while ((ADC14->IFGR0 & ADC14_IFGR0_IFG0))
        degree = ADC14->MEM[0];
}

void EUSCIA0_IRQHandler(void)
{
    if(EUSCI_A_IFG_RXIFG & EUSCI_A0->IFG )
    {
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        x = EUSCI_A0->RXBUF;
    }
    EUSCI_A0->TXBUF = x;
}
