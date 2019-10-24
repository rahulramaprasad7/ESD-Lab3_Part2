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

    P2->DIR |= 0xFF; P2->OUT = 0;
    P3->DIR |= 0xFF; P3->OUT = 0;
    P4->DIR |= 0xFF; P4->OUT = 0;
    P5->DIR |= 0xFF; P5->OUT = 0;
    P6->DIR |= 0xFF; P6->OUT = 0;
    P7->DIR |= 0xFF; P7->OUT = 0;
    P8->DIR |= 0xFF; P8->OUT = 0;
    P9->DIR |= 0xFF; P9->OUT = 0;
    P10->DIR |= 0xFF; P10->OUT = 0;

    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
    __enable_irq();

    while(1)
    {

    }

}
/* Port1 ISR */
void PORT1_IRQHandler(void)
{
    volatile uint32_t i;
    printf("Entered interrupt\n");

    if(P1->IFG & BIT1)
    {
        printf("PWM increase by 10%");
        printf("\n");
        // Delay for P1.1 switch debounce
        for(i = 0; i < 1000; i++)
            P1->IFG &= ~BIT1;
    }



    else if(P1->IFG & BIT4)
    {
        printf("PWM decrease by 10%\n");
        printf("\n");
        // Delay for P1.4 switch debounce
        for(i = 0; i < 1000; i++)
            P1->IFG &= ~BIT4;
    }

}

