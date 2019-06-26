/*
*   7-seg Display Handler for AVR 
*   Encoder Handler for AVR 
*   Dimmer example with zero-crossing
* written  by V. 'Larry' Beliacoff 
*  x3merz@gmail.com
*/


//      func proto
void rotaryEnc();

//*****************************************************************************
// put macros/defines  here
//*****************************************************************************

#define F_CPU 8000000UL
// #define t_pulse_gate 90              //  thyristor/mosfet gate triggering signal length 40 us
#define _FALL 300                          // deadband delay gap
#define _CHA 4700                        // charging side IGBT gate triggering signal length (us)  (4700 def) together with  discharge must not exceed 9400
#define _DIS 4600                       //  discharging side IGBT gate triggering signal length (us)  (4700 def)
#define _FR     250                 // charge frequency
#define delayus _delay_us

//*****************************************************************************
// define pins here
//*****************************************************************************

// gate controlling pins
/**/




#define PORT_L_ON                   PORTC
#define IR_L	                              PINC                        //L input switch
#define PIN_L_ON	                      PC0

#define PORT_N_ON                   PORTC
#define RX_N	                           PINC                        //N input switch
#define PIN_N_ON  	                   PC1

#define PORT_POS_ON               PORTC
#define POS_ON	                       PINC                        //L output switch
#define PIN_POS_ON	               PC2

#define PORT_NEG_ON              PORTC
#define NEG_ON	                      PINC                        //N output switch
#define PIN_NEG_ON	              PC3

// zero crossing detecting pins
#define PORT_ZERO_L               PORTD
#define ZERO_L	                      PIND                        //Detect Zero crossing L
#define PIN_ZERO_L                  PD2

#define PORT_ZERO_N              PORTD
#define ZERO_N	                      PIND                        //Detect Zero crossing N
#define PIN_ZERO_N                  PD3

// other various pins
/*
#define PORT_PWR_LED           PORTC
#define PWR_LED	                      PINC                        //Power LED
#define PIN_PWR_LED               PC4
*/
#define PORT_PWR_LED0           PORTB
#define PWR_LED0	                      PINB                        //Power LED 
#define PIN_PWR_LED0                   PB0

#define PORT_PWR_LED1           PORTB
#define PWR_LED1	                      PINB                        //Power LED1
#define PIN_PWR_LED1                    PB1

#define PORT_PWR_LED2           PORTB
#define PWR_LED2	                      PINB                        //Power LED2
#define PIN_PWR_LED2                    PB2


#define PORT_TRIAC                  PORTD
#define DIR_TRIAC	                    PIND                        
#define PIN_TRIAC                      PD1



//*****************************************************************************
// interfacing a display 

        #define SEGSPLIT   // segment pins on different ports
        #define OMNIDISPL   // display tick timer values
        #define DISPVAL 2 // affects calculation - segment registers (+ 1)
                 /*   TTLSEG = quantity  of segments used, even number   (DISPVAL 2 + 1)    */
        #define TTLSEG 6   //or  ( (DISPVAL 2 + 1) * 2) if TWODISPL defined/two ports used
        #define    TWODISPL   // two sets of registers on two separate ports used
        #define    MAXREGPINS 4
        

        #define PORT_DISPL                    PORTC  // registers port
        #define DIR_DISPL                       DDRC
        #define PORT_SEGM                   PORTB  // segments port
        #define DIR_SEGM                       DDRB
        #define PORT_SEGM_A               PORTD   // 2nd segments port
        #define DIR_SEGM_A                   DDRD         
        
        #define SEGM_PIN0            PB0
        #define SEGM_PIN1            PB1
        #define SEGM_PIN2            PB2
        #define SEGM_PIN3            PB3
        
                     #ifdef   TWODISPL
        #define PORT_DISPL_A                    PORTD
        #define DIR_DISPL_A                       DDRD
        
        #define SEGM_A_PIN0            PD4
        #define SEGM_A_PIN1            PD5
        #define SEGM_A_PIN2            PD6
        #define SEGM_A_PIN3            PD0
			 #else
			 #endif
    


                 //  typedef volatile 
				   struct svnSeg
                         {
                         volatile const uint8_t setPin[MAXREGPINS];
                         volatile   uint8_t *setPort;
                         volatile   uint8_t setVal[DISPVAL * 2 + 2];
                         }  svnSeg, *psvnSeg;

                                           /* array of structures */
                    struct svnSeg displays[] =
                {                 // first display registers
                  {  { SEGM_PIN0, SEGM_PIN1, SEGM_PIN2, SEGM_PIN3 },
	                   &PORT_DISPL  },   // change/increase port/digit pins here
                                                #ifdef   TWODISPL
								  // 2nd display registers
                  {  { SEGM_A_PIN0, SEGM_A_PIN1, SEGM_A_PIN2, SEGM_A_PIN3 },
	                  &PORT_DISPL_A  }
	                                           #endif
                };

                    volatile  uint8_t *setSegmPorts[] = { &PORT_SEGM, &PORT_SEGM_A };
 /* */
                    volatile unsigned char  cDig;        
                   
					volatile static unsigned char      led_s = 1, led_r = 1;

          /*   
 			    common cathode with cathode transistors                              led_s = 1, led_r = 1
 			    common cathode with cathode and anode transistors              led_s = 0, led_r = 1
				common cathode no transistors                                             led_s = 1, led_r = 0
 			    common anode with anode transistors                                   led_s = 0, led_r = 0
 			    common anode  with cathode and anode transistors               led_s = 1, led_r = 0
 			    common anode  no transistors                                              led_s = 0,  led_r = 1
         */

//*****************************************************************************
// define vars here
//*****************************************************************************



  volatile uint16_t counter = 0;
/**/
 volatile uint16_t   display = 0;
 volatile   uint8_t dotIsOn[2];
// #define HUGEDISP
//================
// controller segment pins


   /*    */
#define __PIN0  (1 << 0)   //    0b00000001            
#define __PIN1  (1 << 1)   //    0b00000010           
#define __PIN2  (1 << 2)   //    0b00000100         
#define __PIN3  (1 << 3)  //     0b00001000       
#define __PIN4  (1 << 4)  //     0b00010000         
#define __PIN5  (1 << 5)  //     0b00100000        
#define __PIN6  (1 << 6)  //     0b01000000        
#define __PIN7  (1 << 7) //      0b10000000  

#define A  (1 << 0)   //    0b00000001            
#define B  (1 << 1)   //    0b00000010           
#define C  (1 << 2)   //    0b00000100         
#define D  (1 << 3)  //     0b00001000       
#define E  (1 << 4)  //     0b00010000         
#define F  (1 << 5)  //     0b00100000        
#define G  (1 << 6)  //     0b01000000        
#define DOT  (1 << 7) //  0b10000000  

	
//================

     unsigned char digits[] = {
	(A+B+C+D+E+F),   // 0
	(B+C),           // 1
	(A+B+D+E+G),     // 2
	(A+B+C+D+G),     // 3
	(B+C+F+G),       // 4
	(A+C+D+F+G),     // 5
	(A+C+D+E+F+G),   // 6
	(A+B+C),         // 7
	(A+B+C+D+E+F+G), // 8
	(A+B+C+D+F+G),   // 9
	(A+B+C+E+F+G),   // A - 10
	(C+D+E+F+G),     // b - 11
	(A+D+E+F),       // C - 12
	(B+C+D+E+G),     // d - 13
	(A+D+E+F+G),     // E - 14
	(A+E+F+G),       // F - 15
	(G),             // 16 
	(A+B+F+G),       // 17 
	(0),             // 18 
	(C+E+G),         // n
	(D+E+F+G)        // t
};
///  0, 1, 2, 3, 0, 1, 2, 3


             volatile int8_t firingAngle = 0;
             volatile int8_t  halfwave = 0;
             volatile     static  uint8_t  triacIsOn;
             volatile     static  uint16_t  scaler;



// rotary encoder

uint8_t isPlus = 0;
uint8_t isRev = 0;

// #define BY_NIBBLE
// #define _REVERSE



#define encPort (PINC & encPins)
const int8_t encPins = 0b00110000;
const int8_t encMask = 0b00000011; 

#ifndef BY_NIBBLE
#ifdef _REVERSE
const int8_t encPlus = 0b00101101; 
const int8_t encMinus = 0b00011110;
//  // stepper motor encoder exclusive masks
const int8_t revMinus = 0b01110100;
const int8_t revPlus = 0b01000111;
#else
const int8_t encPlus = 0b00011110; 
const int8_t encMinus = 0b00101101;
//  // stepper motor encoder exclusive masks
const int8_t revMinus = 0b01000111;
const int8_t revPlus = 0b01110100;
#endif
#endif

#ifdef BY_NIBBLE
const int8_t encPlus = 0b01111000;
const int8_t encMinus = 0b01001011;
#endif

