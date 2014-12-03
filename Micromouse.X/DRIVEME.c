
/* Include Header Files */
#include <p32xxxx.h>
#include <plib.h>
#include <xc.h>

// Configuration Bits
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
                                    // see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select
#pragma config JTAGEN = OFF         // Disable JTAG
#pragma config FSOSCEN = OFF        // Disable Secondary Oscillator

//PIN 2  = AN0, Left eye
//PIN 3  = AN1, Middle eye
//PIN 6  = AN2, Right eye

// Defines
#define SYSCLK  40000000L
#define DUTY_CYCLE  97

// Defines
#define AINPUTS 0xffec              // This set A0, A1, A4 as inputs
#define lEYE 4                      // Define the ch to see
#define mEYE 1                      // Define the ch to see
#define rEYE 0                      // Define the ch to see

volatile unsigned int LFT_EYE;      // Connected to AN0 (depending on device)
volatile unsigned int RHT_EYE;      // Connected to AN1 (depending on device)
volatile unsigned int MID_EYE;      // Connected to AN4 (depending on device)

volatile unsigned int vLFT_EYE = 0; // Voltage of LEFT eye
volatile unsigned int vRHT_EYE = 0; // Voltage of RIGHT eye
volatile unsigned int vMID_EYE = 0; // Voltage of MIDDLE eye

volatile int Rcount = 0;
volatile int Lcount = 0;

volatile int STATE = 0; // STATE MACHINE
volatile int mode = 0;  // FOR SETTING SPEED
volatile int RIGHT;     // DIRECTIONAL BIT R
volatile int LEFT;      // DIRECTIONAL BIT L

// SET FREQUENCIES
volatile int FM_PWMR = 7000;
volatile int FM_PWML = 7000;
volatile int run_count;
int SEARCH = 10; // 16 = 13kHz, 32 = 17kHz, 48 = 23kHz // NEW 16 = 24kHZ
                 // 10 = 18 KHz
int counterL = 0;
int counterR = 0;
int EC = 0;         // EC

void initADC(int amask)
{
    AD1CHS = amask;         // select analog input pins
    AD1CON1 = 0x00E0;       // automatic conversion after sampling
    AD1CSSL = 0;            // no scanning required
    AD1CON2 = 0;            // use MUXA, use AVdd & AVss as Vref+/-
    AD1CON3 = 0x1F3F;       // Tsamp = 32 x Tad; 15ksps
  //  AD1CON3 = 0x0080;       // Tsamp = 2 x Tad; sampling at 1000ksps
    AD1CON1bits.ADON = 1;   // turns on the ADC
} //initADC

int readADC(int ch)
{
    AD1CHSbits.CH0SA = ch;      // 1. select input channel
    AD1CON1bits.SAMP = 1;       // 2. start sampling
    while (!AD1CON1bits.DONE);  // 3. wait conversion complete
    AD1CON1bits.DONE = 0;       // 4. clear the flag
    return ADC1BUF0;            // 5. read conversion result
}

 void DelayMs(WORD delay)
 {
     unsigned int int_status;
     while( delay-- )
     {
         int_status = INTDisableInterrupts();
         OpenCoreTimer(SYSCLK / 2000);
         INTRestoreInterrupts(int_status);
         mCTClearIntFlag();
         while( !mCTGetIntFlag() );
     }
     mCTClearIntFlag();
 }

void SET_INOUTS()
 {
    ANSELBbits.ANSB3 = 0; //THIS IS NEED TO TURN OFF THE ANALOG SIDE FOR RB3...
    ANSELAbits.ANSA0 = 0; //AN0 LEFT
    ANSELAbits.ANSA1 = 0; //AN1 MIDDLE
    ANSELBbits.ANSB2 = 0; //AN4 RIGHT

    RPB7Rbits.RPB7R = 0x0005; // PIN 16 FOR STEP 1 INPUT TO DRIVER USES OC1
    RPB8Rbits.RPB8R = 0x0005; // PIN 17 FOR STEP 2 INPUT TO DRIVER USES OC2

    //setting pin 11,14,18 as outputs
    TRISBbits.TRISB4 = 0;	// PIN 11 SET TO RESET OF DRIVER
    TRISBbits.TRISB5 = 0;	// PIN 14 SET DIR for RIGHT MOTOR
    TRISBbits.TRISB9 = 0;	// PIN 18 SET DIR for LEFT MOTOR

    TRISBbits.TRISB15 = 0;	// PIN 26 RUN_LED
    TRISBbits.TRISB14 = 0;	// PIN 25 SPEED_LED
    TRISBbits.TRISB13 = 0;	// PIN 24 SOLVE_LED
    TRISBbits.TRISB11 = 0;	// PIN 22 BREAK_LED

    CNCONAbits.ON = 1;
    CNCONBbits.ON = 1;

    CNENAbits.CNIEA2 = 1;
    CNENAbits.CNIEA3 = 1;
    CNENAbits.CNIEA4 = 1;
    CNENBbits.CNIEB3 = 1;

    CNPDBbits.CNPDB3 = 1;       //PULL DOWNS FOR RUN SWITCH
    CNPDAbits.CNPDA2 = 1;       //PULL DOWNS FOR SPEED SWITCH
    CNPDAbits.CNPDA3 = 1;       //PULL DOWNS FOR SOLVE SWITCH
    CNPDAbits.CNPDA4 = 1;       //PULL DOWNS FOR BREAK SWITCH

    TRISBbits.TRISB3 = 1;	// PIN 07 RUN_SW
    TRISAbits.TRISA2 = 1;	// PIN 09 SPEED_SW
    TRISAbits.TRISA3 = 1;	// PIN 10 SOLVE_SW
    TRISAbits.TRISA4 = 1;	// PIN 12 BREAK_SW

    //setting the output either High or LOW
    LATBbits.LATB4 = 0; 	// PIN 11 Setting Reset of DRIVER 0=OFF;1=ON
    LATBbits.LATB5 = 0; 	// PIN 14 DIR for LEFT MOTOR (DEFault = CCW = 0; CW = 1)
    LATBbits.LATB9 = 1; 	// PIN 18 DIR for RIGHT MOTOR (DEFault = CW = 1; CCW = 0)

    LATBbits.LATB15 = 0; 	// PIN 26 RUN_LED
    LATBbits.LATB14 = 0; 	// PIN 25 SPEED_LED
    LATBbits.LATB13 = 0; 	// PIN 24 SOLVE_LED
    LATBbits.LATB11 = 0; 	// PIN 22 BREAK_LED

    return;
 }

 void init()
{
    // Configure standard PWM mode for output compare module 1

    //    T1CONSET = 0x8000;  // Enable Timer1, prescaler 1:1
    //    T1CONSET = 0x8010;  // Enable Timer1, prescaler 1:8
    //    T1CONSET = 0x8020;  // Enable Timer1, prescaler 1:64

    PR1 = (SYSCLK / (2 * 7000)) + 1;
    T1CONSET = 0x8030;  // Enable Timer1, prescaler 1:256

    // Set OC1 to pin RB7 with peripheral pin select
    OC1CON = 0x0005;    // Initialize OCx pin low;
                        // Generate continuous output pulses on OC1 pin
    T2CONSET = 0x8010;  // Enable Timer2, prescaler 1:2
    OC1CONSET = 0x8000; // Enable Output Compare Module 1
                        // DEFAULT is timer 1 enabled.

    // Configure standard PWM mode for output compare module 2
    OC2CON = 0x0005;      // Initialize OCx pin low;
                          // Generate continuous output pulses on OC1 pin
    T3CONSET = 0x8010;    // Enable Timer3, prescaler 1:2
    OC2CONSET = 0x8008;   // Enable Output Compare Module 2
                          // Needs to have timer 3 enabled
    return;
}

int main(void)
{
    SYSTEMConfigPerformance(SYSCLK);
    SYSTEMConfig(SYSCLK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    initADC(AINPUTS);

    INTEnable(INT_CNB, INT_ENABLED);
    INTEnable(INT_CNA, INT_ENABLED);
    INTSetVectorPriority(INT_CHANGE_NOTICE_VECTOR, INT_PRIORITY_LEVEL_7);
    INTEnableSystemMultiVectoredInt();

    mT1SetIntPriority(2); // 2
    mT1IntEnable(1);

    SET_INOUTS();

    while(1) // Need for mapping
    {
        //jump here and do nothing
    } //while -- Main Loop


}//main

void DIR_GO(int RIGHT, int LEFT)
{
    LATBbits.LATB9 = RIGHT; // DIR for RIGHT MOTOR (DEFault = CCW = 0; CW = 1)
    LATBbits.LATB5 = LEFT;  // DIR for LEFT MOTOR  (DEFault = CW = 1; CCW = 0)
    return;
}


//void DIR180()
//{
//    if( vRHT_EYE > vLFT_EYE)
//    {
//      DIR_GO(0, 0); // RIGHT
//          DelayMs(160); // After Turn  100
//          DIR_GO(1,0);  // STRAIGHT
//          DelayMs(180); // Going Straight 200
//          STATE = 0;
//          return;
//     }
//     if( vRHT_EYE < vLFT_EYE)
//    {
//      DIR_GO(1, 1); // LEFT
//          DelayMs(160); // After Turn  100
//          DIR_GO(1,0);  // STRAIGHT
//          DelayMs(180); // Going Straight 200
//          STATE = 0;
//          return;
//     }
//}

void DIR90L()
{
    DIR_GO(1, 1); // LEFT
    DelayMs(114); // After Turn  100
    DIR_GO(1,0);  // STRAIGHT
    DelayMs(110);  // Going Straight 200
    STATE = 0;
    return;
}
void DIR90LN()
{
    DelayMs(43);
    DIR_GO(1, 1); // LEFT
    DelayMs(114); // After Turn  100
    DIR_GO(1,0);  // STRAIGHT
    DelayMs(108);  // Going Straight 200
    STATE = 0;
    return;
}
void DIR90R()
{
    DIR_GO(0, 0); // RIGHT
    DelayMs(98); // After Turn  100
    DIR_GO(1,0);  // STRAIGHT
    DelayMs(107);  // Going Straight 200
    STATE = 0;
    return;
}
void DIR90RN()
{
    DelayMs(90);
    DIR_GO(0, 0); // RIGHT
    DelayMs(98); // After Turn  100
    DIR_GO(1,0);  // STRAIGHT
    DelayMs(108);  // Going Straight 200
    STATE = 0;
    return;
}

void SET_SPEED(int roffset, int loffset, int rdoffset, int ldoffset, int mode)
{
    if (mode == 0)                                          // ACCELERATION (NOT USED FOR EC)
    {
        PR2 = (SYSCLK / (2 * FM_PWMR)) + 1;
        OC1RS = (PR2 - 1) * ((float)DUTY_CYCLE / 100);

        if (rdoffset == 0) FM_PWMR = FM_PWMR + roffset;

        PR3 = (SYSCLK / (2 * FM_PWML)) + 1;
        OC2RS = (PR3 - 1) * ((float)DUTY_CYCLE / 100);

        if (ldoffset == 0) FM_PWML = FM_PWML + loffset;
    }

    else if (mode == 1)                                     // CONSTANT. ERROR CORRECT ALSO
    {
        if (rdoffset == 0)
            FM_PWMR = FM_PWMR + roffset;
        else if (rdoffset == 1)
            FM_PWMR = FM_PWMR - roffset;

        PR2 = (SYSCLK / (2 * FM_PWMR)) + 1;
        OC1RS = (PR2 - 1) * ((float)DUTY_CYCLE / 100);
        PR3 = (SYSCLK / (2 * FM_PWML)) + 1;
        OC2RS = (PR3 - 1) * ((float)DUTY_CYCLE / 100);
    }
    return;
}

// Configure the Timer 1 interrupt handler
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
        MID_EYE = readADC(mEYE);        // Looks for values at the Middle sensor
        RHT_EYE = readADC(rEYE);        // Looks for values at the Right sensor
        LFT_EYE = readADC(lEYE);        // Looks for values at the Left sensor

        vLFT_EYE = ((3300) * LFT_EYE) >> 10; // Converts into a voltage value
        vRHT_EYE = ((3300) * RHT_EYE) >> 10; // Converts into a voltage value
        vMID_EYE = ((3300) * MID_EYE) >> 10; // Converts into a voltage value

        //////////////////////////////// NOTES /////////////////////////////////
        //                                                                    //
        //   4 CASE DIRECTIONS: (0) STRAIGHT, (1) LEFT, (2) RIGHT, (3) 180    //
        //     BIT ASSIGNMENTS:       (1,0)       (1,1)     (0,0)             //
        // DIR for RIGHT MOTOR:   (Default: CCW = 0; CW = 1)                  //
        //  DIR for LEFT MOTOR:   (Default: CW = 1; CCW = 0)                  //
        //                                                                    //
        //  SET_SPEED (Roffset, Loffset, rdoffset, ldoffset, mode)            //                                                                //
        ////////////////////////////////////////////////////////////////////////

        if (Rcount > SEARCH && Lcount > SEARCH) // ERROR CORRECTION
        {
            if (vRHT_EYE > 1079) // VEERS LEFT
            {
                SET_SPEED(2000, 0, 1, 0, 1);   // Close to Left Wall. Decrease RIGHT Motor 7
                if (FM_PWMR < 17500) // 18500
                    FM_PWMR = 17500; // Keepin speed at 23 or 24kHz
            }

            if (vLFT_EYE > 1120) // VEERS RIGHT
            {
                SET_SPEED(2000, 0, 0, 0, 1);   // Close to Right Wall. Increase RIGHT Motor 7
                if (FM_PWMR > 18500) // 17500
                    FM_PWMR = 18500; // Keepin speed at 23 or 24kHz
            }

            
        } // END ERROR CORRECTION

        // CONDITIONS ///////////////////////////////////////////////////////////////////////
//        if (vLFT_EYE > 600 && vRHT_EYE > 600)      // WALL LEFT/RIGHT (1)
//        {
//            STATE = 0;       // STRAIGHT
//        }
//
//        else if (vLFT_EYE > 600 && vMID_EYE < 700 && vRHT_EYE < 600)  // WALL LEFT (2)
//        {
//            STATE = 0;      // RIGHT 2
//        }
//
//        else if (vLFT_EYE < 600 && vMID_EYE < 700 && vRHT_EYE > 600)  // WALL RIGHT (3)
//        {
//            STATE = 0;      // LEFT 1
//        }

        if (vMID_EYE > 780 && vRHT_EYE < 600 && vLFT_EYE > 600)  // WALL LEFT AND MID
        {
            STATE = 2;      // RIGHT
        }

        if (vMID_EYE > 780 && vRHT_EYE > 600 && vLFT_EYE < 600)  // WALL RIGHT AND MID
        {     
            STATE = 1;      // LEFT
        }
       if (vMID_EYE < 300 && vRHT_EYE < 600 && vLFT_EYE > 600)  // WALL LEFT AND MID
        {
            STATE = 3;      // RIGHT
        }
         if (vMID_EYE < 300 && vRHT_EYE > 600 && vLFT_EYE < 600)  // WALL RIGHT AND MID
        {
            STATE = 4;      // LEFT
        }

//        if (vMID_EYE > 800 && vRHT_EYE > 600 && vLFT_EYE > 600)  // WALL LEFT AND MID (5)
//        {
//
//
//        STATE = 3;      // RIGHT --2
//        }
//
//        else if (vLFT_EYE < 600 && vMID_EYE > 700 && vRHT_EYE > 600)  // WALL RIGHT AND MID (6)
//        {
//            STATE = 1;      // LEFT --1
//        }
//
//        else if (vLFT_EYE > 600 && vMID_EYE > 700 && vRHT_EYE > 600)  // WALL LEFT, RIGHT AND MID (7)
//        {
//            STATE = 0;      // 180 --> Once called, even no wall in front, makes it turn right and left...breaks
//        }
        // END OF CONDITIONS //////////////////////////////////////////////////////////////////////////

        /////////////////////////////// FSM /////////////////////////////////
        //                                                                 //
        //  EACH CASE CORRELATES TO DIRECTION: STRAIGHT, LEFT, RIGHT, 180  //
        //                                                                 //
        /////////////////////////////////////////////////////////////////////

        switch (STATE)  // Perform actions and set conditions of each state
        {
            case 0: // STRAIGHT
                    DIR_GO(1, 0);
                    if (Rcount <= SEARCH && Lcount <= SEARCH)
                        {
                            SET_SPEED(1000, 1000, 0, 0, 0); // ACCEL
                            Rcount++;
                            Lcount++;
                        }
                    if (Rcount == SEARCH && Lcount == SEARCH)
                    {
                            SET_SPEED(1000, 1000, 0, 0, 1); // CONST VEL
                            FM_PWMR = FM_PWML;
                    }
                    break;

            case 1: // LEFT
                    DIR90L();
                    break;

            case 2: // RIGHT
                    DIR90R();
                    break;

            case 3: // RIGHT
                    DIR90RN();
                    break;
            case 4: // LEFT
                    DIR90LN();
                    break;
//            case 3: // 180
//
//                DIR180();
//
//                break;
//             Rcount++;
//             Lcount++;

        mT1ClearIntFlag();
        } // END SWITCH
} // END TIMER 1


void __ISR(_CHANGE_NOTICE_VECTOR, ipl7) ChangeNotice_Handler (void)
{
      // SWITCHES AND PUSH BUTTONS /////////////////////////////////////////////
      if(PORTBbits.RB3 == 1)
      {
          LATBbits.LATB4 = 1; 	// PIN 11 Setting Reset of DRIVER 0=OFF;1=ON
          LATBbits.LATB15 = 1; 	// PIN 26 RUN_LED
          run_count = 1;
          Rcount = Lcount = 0;
          FM_PWMR = FM_PWML = 7000;
          init();
      }
      else if(PORTBbits.RB3 == 0) LATBbits.LATB15 = 0;

      if(PORTAbits.RA2 == 1)
      {
          run_count = 2;
          LATBbits.LATB14 = 1; 	// PIN 25 SPEED_LED
      }
      else if(PORTAbits.RA2 == 0) LATBbits.LATB14 = 0;

      if(PORTAbits.RA3 == 1)
      {
          run_count = 3;
          LATBbits.LATB13 = 1; 	// PIN 24 SOLVE_LED
      }
      else if(PORTAbits.RA3 == 0) LATBbits.LATB13 = 0;

      if(PORTAbits.RA4 == 1)
      {
          run_count = 4;
          LATBbits.LATB11 = 1; 	// PIN 22 BREAK_LED
      }
      else if(PORTAbits.RA4 == 0) LATBbits.LATB11 = 0;

      if (run_count == 1) //runs
      {
          // this will enable motors and make it go.....
          LATBbits.LATB15 = 1;// PIN 26 RUN_LED
          DelayMs(100);
          LATBbits.LATB15 = 0;// PIN 26 RUN_LED
          DelayMs(100);
      }
      if (run_count == 2)// speed
      {
          //to change freq or mode id needed
          LATBbits.LATB14 = 1; 	// PIN 25 SPEED_LED
          DelayMs(50);
          LATBbits.LATB14 = 0; 	// PIN 25 SPEED_LED
          DelayMs(50);
      }
      if (run_count == 3)// solve
      {
          //THIS IS THE MAPPING AND SOLVING SECTION
          LATBbits.LATB13 = 1;// PIN 24 SOLVE_LED
          DelayMs(150);
          LATBbits.LATB13 = 0;// PIN 24 SOLVE_LED
          DelayMs(150);
      }

      if (run_count == 4)//break
      {
          LATBbits.LATB4 = 0; 	// PIN 11 Setting Reset of DRIVER 0=OFF;1=ON
          LATBbits.LATB15 = 1;  // PIN 26 RUN STAYS ON
          LATBbits.LATB14 = 1; 	// PIN 25 SPEED_LED
          LATBbits.LATB13 = 1;  // PIN 24 SOLVE_LED
          LATBbits.LATB11 = 1; 	// PIN 22 BREAK_LED
          DelayMs(40);
          LATBbits.LATB15 = 0;  // PIN 26 RUN STAYS ON
          LATBbits.LATB14 = 0; 	// PIN 25 SPEED_LED
          LATBbits.LATB13 = 0;  // PIN 24 SOLVE_LED
          LATBbits.LATB11 = 0; 	// PIN 22 BREAK_LED
          DelayMs(40);
      }
    mCNAClearIntFlag(); // clear the interrupt flag
    mCNBClearIntFlag(); // clear the interrupt flag

} // END ISR AND PB














