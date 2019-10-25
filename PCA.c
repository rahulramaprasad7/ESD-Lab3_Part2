#include <mcs51/at89c51ed2.h>
#include <stdio.h>

#define HEAP_SIZE 4000
__xdata char __sdcc_heap[HEAP_SIZE];
const unsigned int __sdcc_heap_size = HEAP_SIZE;   //Set the Heap size to 4000 bytes

/* Referred to the application note "FlashFlex Microcontroller Using the Programmable Counter Array (PCA)" */

int flag = 0; // Rising or falling edge flag.
unsigned int pulse_width = 0x00; // Final pulse width calculation is stored here.
unsigned int capture1 = 0x00; // Rising edge captured here.
unsigned int capture2 = 0x00; // Falling edge captured here.

char input;

void idle_mode()
{
    PCON |= 0x01;    //Setting the IDL bit in PCON
}

void powerDown_mode()
{
    PCON |= 0x02;    //Setting the PD bit in PCON
}
void delay()
{
    unsigned int i;
    for(i = 0; i < 1024; i++);   //Delay for Watchdog
}
void refresh_WDT()
{
    EA = 0;
    CCAP4L = 0;
    CCAP4H = CH;
    EA = 1;
}

_sdcc_external_startup()
{
    AUXR &= (!XRS2);				//Clear the XRS2 bit
    AUXR |= XRS1 | XRS0;            //Set XRS1 and XRS0 bit to allocate 1024 bytes for XRAM
    return 0;
}

void timer_init()
{
    CMOD = 0x01; // Setup PCA timer mode.
    CH = 0x00; // Init values
    CL = 0x00;
    CCAP0L = 0x20; // Set compare limit
    CCAP0H = 0x4E;
    CCAPM0 = 0x49; // Set Modules zero for 16bit Timer mode.
    IE = 0xC0; // Enable Interrupts CR = 1; // Start PCA timer
}

void highSpeed_init()
{
    CMOD = 0x02; // Setup PCA Timer CL = 0x00; CH = 0x00;
    CCAP0L = 0xFF; // Set Event trigger values
    CCAP0H = 0xFF;
    CCAPM0 = 0x4D; // Set PCA module 0 for HSO mode
    IE = 0xC0; CR = 1; // Start PCA timer.

}

void PCA_ISR() __interrupt (6)
{
    if (CCF1 == 1) //ISR for Timer16
    {
        P1_1 ^= 1;        //Toggle P1.1
        putchar('a');
        unsigned int temp;
        IE = IE & 0xBF; // Stop Interrupts
        CCF1 = 0; // Clear Int
        temp = CCAP0L | (CCAP0H << 8); // The following four lines
        temp += 0x4E20; // of code increase the
        CCAP0L = temp; // compare value in CCAP0
        CCAP0H = temp >> 8; // by 20000, effectively // refreshing the timer.
        IE = IE | 0x40; // Start interrupts
    }
    if (CCF2 == 1)  //ISR for High Speed
    {
        IE &= 0xBF;
        CCF2 = 0;
        CCAP2H += 0x40; //Counter Valye
        IE |= 0x40;
    }
}

void PWM_run()
{
    CMOD = 0x02; // Setup PCA timer mode.
    CH = 0x00; // Init values
    CL = 0x00;
    CCAP0L = 0xB4; // Set compare limit
    CCAP0H = 0xB4; //Duty cycle of 30% (CCAP0H = 256(1 - Duty cycle))
    CCAPM0 = 0x42; // Set Modules zero for PCA
    CR = 1;
}
void PWM_stop()
{
    CMOD = 0x00;  //Clearing the PWM bit
}
 int getchar ()
{
    while ((SCON & 0x01) == 0);  // wait for character to be received, spin on RI
	RI = 0;			// clear RI flag
	return SBUF;  	// return character from SBUF
}
int putchar (int c)
{
	while ((SCON & 0x02) == 0);    // wait for TX ready, spin on TI
	SBUF = c;  	// load serial port with transmit value
	TI = 0;  	// clear TI flag
	return 0;
}
int putstr (char *s)
{
	int i = 0;
    while (*s)  // output characters until NULL found
    {
		putchar(*s++);
		i++;
    }
    putchar('\n');
    return i+1;
}
 void main()
 {
    timer_init(); //Initialise Timer Registers
//    CCAP4L = 0xFF; // Setup PCA module 4 for Watchdog Timer
//    CCAP4H = 0xFF;
//    CCAPM4 = 0x4C;
//    CMOD = CMOD | 0x40;
//    while (1)
//    {
//        refresh_WDT(); // This function refreshes the WDT and should be // used periodically. delay(); }
//    }
     highSpeed_init();  //Initialise Registers for High-Speed Mode
     char message[100] ="PCA control program, press + to start pwm and - to stop pwm, i for idle mode, p for power down mode";
     putstr(message);
     while (1)
     {
         input = getchar();
         putchar(input);
         if ( input == '+') //Start PWM
            PWM_run();
         if (input == '-')  //Stop PWM
            PWM_stop();
         if (input == 'i')  //Enter Idle Mode
            idle_mode();
        if (input == 'p')   //Enter PowerDown Mode
            powerDown_mode();
         putchar('j');
     }
}



