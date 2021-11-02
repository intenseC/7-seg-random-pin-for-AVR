/*
*   7-seg Display Handler for AVR 
*   Encoder Handler for AVR 
*   Dimmer example with zero-crossing
*   written  by Simon  Katznelson  
*   x3merz@gmail.com
*/




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




#define PORT_L_ON                PORTC
#define IR_L	                 PINC                        //L input switch
#define PIN_L_ON	         PC0

#define PORT_N_ON                PORTC
#define RX_N	                 PINC                        //N input switch
#define PIN_N_ON  	         PC1

#define PORT_POS_ON              PORTC
#define POS_ON	                 PINC                        //L output switch
#define PIN_POS_ON	         PC2

#define PORT_NEG_ON              PORTC
#define NEG_ON	                 PINC                        //N output switch
#define PIN_NEG_ON	         PC3

// zero crossing detecting pins
#define PORT_ZERO_L              PORTD
#define ZERO_L	                 PIND                        //Detect Zero crossing L
#define PIN_ZERO_L               PD2

#define PORT_ZERO_N              PORTD
#define ZERO_N	                 PIND                        //Detect Zero crossing N
#define PIN_ZERO_N               PD3

// other various pins
#if 0
#define PORT_PWR_LED            PORTC
#define PWR_LED	                PINC                        //Power LED
#define PIN_PWR_LED             PC4
#endif
#define PORT_PWR_LED0           PORTB
#define PWR_LED0	        PINB                        //Power LED 
#define PIN_PWR_LED0            PB0

#define PORT_PWR_LED1           PORTB
#define PWR_LED1	        PINB                        //Power LED1
#define PIN_PWR_LED1            PB1

#define PORT_PWR_LED2           PORTB
#define PWR_LED2	        PINB                        //Power LED2
#define PIN_PWR_LED2            PB2


#define PORT_TRIAC              PORTD
#define DIR_TRIAC	        PIND                        
#define PIN_TRIAC               PD1
//*****************************************************************************
// display  interfacing

#define SEGSPLIT     // segment pins on different ports
#define OMNIDISPL    // display tick timer values
#define DISPVAL 2    // affects calculation - segment registers (+ 1)
                 /*   TTLSEG = quantity  of segments used, even number   (DISPVAL 2 + 1)    */
#define TTLSEG 6     //or  ( (DISPVAL 2 + 1) * 2) if TWODISPL defined/two ports used
#define TWODISPL     // two sets of registers on two separate ports used
#define MAXREGPINS 4
        

#define PORT_DISPL                  PORTC   // registers port
#define DIR_DISPL                   DDRC
#define PORT_SEGM                   PORTB   // segments port
#define DIR_SEGM                    DDRB
#define PORT_SEGM_A                 PORTD   // 2nd segments port
#define DIR_SEGM_A                  DDRD         
        
#define SEGM_PIN0              PB0
#define SEGM_PIN1              PB1
#define SEGM_PIN2              PB2
#define SEGM_PIN3              PB3
        
#ifdef   TWODISPL
#define PORT_DISPL_A           PORTD
#define DIR_DISPL_A            DDRD
        
#define SEGM_A_PIN0            PD4
#define SEGM_A_PIN1            PD5
#define SEGM_A_PIN2            PD6
#define SEGM_A_PIN3            PD0
#endif
               
struct SvnSeg {
 volatile const uint8_t setPin[MAXREGPINS];
 volatile   uint8_t *setPort;
 volatile   uint8_t setVal[DISPVAL * 2 + 2];
};


/*   
                 Mapping:
 			    common cathode with cathode transistors                        led_s = 1, led_r = 1
 			    common cathode with cathode and anode transistors              led_s = 0, led_r = 1
			    common cathode no transistors                                  led_s = 1, led_r = 0
 			    common anode with anode transistors                            led_s = 0, led_r = 0
 			    common anode  with cathode and anode transistors               led_s = 1, led_r = 0
 			    common anode  no transistors                                   led_s = 0,  led_r = 1
*/


// #define HUGEDISP
//================
// controller segment pins
#define __PIN0  (1 << 0)  //     0b00000001            
#define __PIN1  (1 << 1)  //     0b00000010           
#define __PIN2  (1 << 2)  //     0b00000100         
#define __PIN3  (1 << 3)  //     0b00001000       
#define __PIN4  (1 << 4)  //     0b00010000         
#define __PIN5  (1 << 5)  //     0b00100000        
#define __PIN6  (1 << 6)  //     0b01000000        
#define __PIN7  (1 << 7)  //     0b10000000  

#define A  (1 << 0)   //    0b0000000*            
#define B  (1 << 1)   //    0b000000*0           
#define C  (1 << 2)   //    0b00000*00         
#define D  (1 << 3)   //    0b0000*000       
#define E  (1 << 4)   //    0b000*0000         
#define F  (1 << 5)   //    0b00*00000        
#define G  (1 << 6)   //    0b0*000000        
#define DOT  (1 << 7) //    0b*0000000  

// #define BY_NIBBLE
// #define _REVERSE

#define encPort (PINC & encPins)
