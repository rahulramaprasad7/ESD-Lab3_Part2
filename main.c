#include "msp.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Referred to the temperature sensor example for calibrating the sensor */

int check =0;  //Flag to take pushbutton input
volatile uint32_t temp;
uint16_t x;  //USing to echo character
char buffer[10];  //Buffer to output serial message
uint16_t userPWM; //Using to store user input for duty cycle


char startMessage[80] = "Enter p to check duty cycle, t for temperature ";
char pwmMessage[20] = "PWM Duty Cycle = ";
char unitMessage[20] = "Changing the Unit ";
char pwmUserMessage[20] = "Wrong Input Value ";
char tempMessage[20] = "Temperature is ";
char newLine [2] = {'\n','\r'};
char celsius [2];
char fahreneit[2];
char percent [2];

uint16_t pwmLevel = 4;


#define PWMCOUNT 6553  //10% of 65530

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
    P1->DIR |= ~(uint8_t) BIT1 | ~(uint8_t) BIT4;   //Enabling pushbuttons as input
    P1->OUT |= BIT1 | BIT4;                         //Selecting pullup resistors as alternate function of OUT register
    P1->REN |= BIT1 |BIT4;                          // Enable pull-up resistor (P1.1 output high)
    P1->SEL0 = 0;
    P1->SEL1 = 0;
    P1->IES |= BIT1 | BIT4;                         // Interrupt on high-to-low transition
    P1->IFG = 0;                                    // Clear all P1 interrupt flags
    P1->IE |= BIT1 | BIT4;                          // Enable interrupt for P1.
}

void uart_init()
{

    P1->SEL0 |= BIT2 | BIT3;  //Selecting the alternate functions of P1.2 and P1.3 for Rx and Tx

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;  //Enabling software reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;  //Enabling clock
    /* Baudrate calculation
     * Fclk = 3Mhz, Baudrate = 460800
     * N = Fclk/ BaudRate = 6.51
     * BR = 6
     * BRS = (6 - 6.51) = 0.51 which approximately maps to 0xAA
     */
    EUSCI_A0->BRW = 6;
    EUSCI_A0->MCTLW = (0xAA << EUSCI_A_MCTLW_BRS_OFS);

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;  //Disabling reset
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;      //Clearing Rx flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;          //Enabling Rx interrupt
}

void timer_init()
{
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_6 | TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled and output mode 6
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_OUTMOD_6 | TIMER_A_CCTLN_CCIE; // TACCR1 interrupt enabled and output mode 6

    TIMER_A0->CCR[0] = 65531;  //Initialising Timer_A Compare 0 register
    TIMER_A0->CCR[1] = 4 * PWMCOUNT;  //Initialising 40 % duty cycle

    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__CONTINUOUS; // SMCLK, continuous mode

}
void putstr (char *buff)
{
    int i = 0;
    while(buff[i] != '\0') //Transmit until the NULL character is reached
    {
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); //Wait till other interrupts are cleared
        EUSCI_A0->TXBUF = buff[i];  //Transmit by oading into TxBUF
        i++;
    }
}
void main(void)
{
    bool unitFlag = false;
    int32_t adcRefTempCal_1_2v_30;
    int32_t adcRefTempCal_1_2v_85;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    snprintf(percent, 2, "%c", '%');      //Converting the % character to transmit serially
    snprintf(celsius, 2, "%c", 'C');      //Converting the C character to transmit serially
    snprintf(fahreneit, 2, "%c", 'F');   //Converting the F character to transmit serially

    adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;    //Accessing calibrated value for 30 degree Celsius
    adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;    //Accessing calibrated value for 85 degree Celsius

    temp_init();  //Initialising Temperature sensor registers
    timer_init(); //Initialising Timer registers
    push_init();  //Initialising push buttons
    uart_init();  //Initialising UART

    P1->DIR |= BIT0;  //Enabling output
    P1->OUT |= BIT0;  //Setting the output

    __enable_irq();   //Enable global interrupts

    NVIC->ISER[1] |= 1 << ((PORT1_IRQn) & 31);
    NVIC->ISER[0] |= (1 << (EUSCIA0_IRQn & 31)) | (1 << (TA0_0_IRQn & 31)) | (1 << (TA0_N_IRQn & 31)) | (1 << ((ADC14_IRQn) & 31)) ;

    putstr(startMessage);
    putstr(newLine);
    while(1)
    {
        if ( x == 't')
        {
            ADC14->CTL0 |= ADC14_CTL0_SC;  //Starting the ADC conversion
            IntDegC = (((float) degree - adcRefTempCal_1_2v_30) * (85 - 30)) / (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) - 12.5; //Calibrating the values
            IntDegF = ((9 * IntDegC) / 5) + 32; //Conversion from Celsius scale to Fahreneit scale

            putstr(tempMessage);

            if (!(unitFlag))
            {
                snprintf(buffer,10,"%f", IntDegC);
                putstr(buffer);
                putstr(celsius);
            }
            if (unitFlag)
            {
                snprintf(buffer,10,"%f", IntDegF);
                putstr(buffer);
                putstr(fahreneit);
            }
            putstr(newLine);
            memset(buffer, '\0', 10*sizeof(char)); //Reset the Buffer
            x = NULL; //Reset the character used to echo
        }

        if( x == 'p')
        {
            snprintf(buffer,10,"%u", (pwmLevel * 10)); //Print Duty cycle
            putstr(pwmMessage);
            putstr(buffer);
            putstr(percent);
            putstr(newLine);
            memset(buffer, '\0', 10*sizeof(char)); //Reset Buffer
            x = NULL; //Reset character used to echo
        }

        if(x == 'c')
        {
            while((EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                userPWM= EUSCI_A0->RXBUF;
            if(userPWM >=0 && userPWM <= 10)
            {
                pwmLevel = userPWM;
            }
            else
            {
                putstr(pwmUserMessage);
                putstr(newLine);
            }
        }
        if ( x == 'u')
        {
            putstr(unitMessage);
            putstr(newLine);
            unitFlag ^= true;
            memset(buffer, '\0', 10*sizeof(char)); //Reset Buffer
            x = NULL; //Reset character used to echo
        }

        if (check == 1) //If Pushbutton 1 is pressed
        {
            if(pwmLevel != 0) //Check if pwmLevel is 0
            {
                if (TIMER_A0->CCR[1] > 0) //Check if TACCR1 is 0
                {
                    pwmLevel--;  //Reduce the duty cycle
                    TIMER_A0->CCR[1] = pwmLevel * PWMCOUNT;  //Reduce the duty cycle by reducing count
                }
                check = 0;  //Clear the flag
            }
        }
        if (check == 2) //If Pushbutton 2 is pressed
        {
            if(pwmLevel != 10) //Check if pwmLevel is 10
            {
                if (TIMER_A0->CCR[1] <65530) //Check if TACCR1 is 65530
                {
                    pwmLevel++; //Increase duty cycle
                    TIMER_A0->CCR[1] = pwmLevel * PWMCOUNT; //Increase duty cycle by 10 % by increasing count
                }
                check = 0; //Clear the flag
            }
        }
    }

}
/* Port1 ISR */
void PORT1_IRQHandler(void)
{
    volatile uint32_t i;

    if(P1->IFG & BIT1) //Check for Pushbutton 1
    {
        // Delay for P1.1 switch debounce
        for(i = 0; i < 5000; i++)
            P1->IFG &= ~BIT1;
        check = 1;  //Set the flag to decrease by 10%
    }

    else if(P1->IFG & BIT4)  //Check for PushButton 2
    {
        // Delay for P1.4 switch debounce
        for(i = 0; i < 5000; i++)
            P1->IFG &= ~BIT4;
        check = 2;   //Set the flag to increase by 10%
    }

}

void TA0_0_IRQHandler(void)
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT |= BIT0;   //Set the output after TACCR0 is reached
}

void TA0_N_IRQHandler(void)
{
    TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT &= ~BIT0;  //Reset the output after TACCR1 is reached
}

void ADC14_IRQHandler(void)
{
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG0)
    {
        degree = ADC14->MEM[0];  //Store the ADC sample
    }
}

void EUSCIA0_IRQHandler(void)
{
    if(EUSCI_A_IFG_RXIFG & EUSCI_A0->IFG )
    {
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        x = EUSCI_A0->RXBUF;  //Receive a character
    }
    EUSCI_A0->TXBUF = x;  //Transmit the character
}


