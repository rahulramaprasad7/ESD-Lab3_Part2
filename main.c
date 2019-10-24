#include "msp.h"
#include <stdio.h>

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	P1->DIR |= ~(uint8_t) BIT1 | ~(uint8_t) BIT4;
    P1->OUT |= BIT1 | BIT4;
    P1->REN |= BIT1 |BIT4;                          // Enable pull-up resistor (P1.1 output high)
    P1->SEL0 = 0;
    P1->SEL1 = 0;
    P1->IES |= BIT1 | BIT4;                         // Interrupt on high-to-low transition
    P1->IFG = 0;                                    // Clear all P1 interrupt flags
    P1->IE |= BIT1 | BIT4;                          // Enable interrupt for P1.1

    P1->DIR |= BIT0;
    P1->OUT |= BIT0;

    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_OUTMOD_3 | TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_3 | TIMER_A_CCTLN_CCIE; // TACCR1 interrupt enabled

    TIMER_A0->CCR[0] = 18750;
    TIMER_A0->CCR[1] = 18750;

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP; // SMCLK, up mode

    TIMER_A0->CTL |= TIMER_A_CTL_ID_2;       //Pre-scaler divide by 4
    TIMER_A0->EX0 |= TIMER_A_EX0_TAIDEX_7;   //Prescaler divide by 8

    P2->DIR |= 0xFF; P2->OUT = 0;
    P3->DIR |= 0xFF; P3->OUT = 0;
    P4->DIR |= 0xFF; P4->OUT = 0;
    P5->DIR |= 0xFF; P5->OUT = 0;
    P6->DIR |= 0xFF; P6->OUT = 0;
    P7->DIR |= 0xFF; P7->OUT = 0;
    P8->DIR |= 0xFF; P8->OUT = 0;
    P9->DIR |= 0xFF; P9->OUT = 0;
    P10->DIR |= 0xFF; P10->OUT = 0;

    NVIC->ISER[1] |= 1 << ((PORT1_IRQn) & 31);
    NVIC->ISER[0] |= (1 << (TA0_0_IRQn & 31)) | (1 << (TA0_N_IRQn & 31));

    __enable_irq();

    while(1)
    {

    }

}
/* Port1 ISR */
void PORT1_IRQHandler(void)
{
    volatile uint32_t i;

    if(P1->IFG & BIT1)
    {
        //printf("PWM increase by 10% ");
        TIMER_A0->CCR[0] += (18750 + 1875);
        //printf("%d",TIMER_A0->CCR[0]);
        //printf("\n");
        // Delay for P1.1 switch debounce
        for(i = 0; i < 10000; i++)
            P1->IFG &= ~BIT1;
    }



    else if(P1->IFG & BIT4)
    {
        //printf("PWM decrease by 10% ");
        TIMER_A0->CCR[1] += (18750);
        //printf("%d",TIMER_A0->CCR[1]);
        //printf("\n");
        // Delay for P1.4 switch debounce
        for(i = 0; i < 10000; i++)
            P1->IFG &= ~BIT4;
    }

}

void TA0_0_IRQHandler(void)
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT ^= BIT0;
}

void TA0_N_IRQHandler(void)
{
    TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT ^= BIT0;
}
