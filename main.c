/*
*   7-seg Display Handler for AVR; 
*   Encoder Handler for AVR; 
*   AC Dimmer example with 
*   zero-crossing detection.
*   written  by Simon  Katznelson 
*   x3merz@gmail.com
*/



//*****************************************************************************
//place includes here
//*****************************************************************************
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>

#include <avr/interrupt.h>
#include "main.h"
// #include <math.h>



//*****************************************************************************
//*****************************************************************************

void init_io(void) 
{
	/* first set all pins to input */

     DIR_DISPL = 0b00001111;                       /* 0123 as output  */
            PORT_DISPL = 0b00110000;                         /* 4-5 pulled rest are low  */
     DIR_SEGM = 0xFF; //  /* all pins output  */
     PORT_SEGM = 0x00;
              #ifdef   TWODISPL
            PORT_DISPL_A = 0x00;     /* all pins low  */ 
			 #endif
         DIR_DISPL_A = 0b11111011;        /* INT0 as input  */
	

	
}
 //*****************************************************************************
 //*****************************************************************************    
void
timer_init (void)
{
	 cli();                    // global disable interrupts


       TCCR0 |= (1 << CS02) | (0 << CS01) | (1 << CS00);            //enable timer0 with 1024 prescaler
       TCNT0 = 0;                                                                       //reset count
   
	        MCUCR |= (1 << ISC01);        // interrupt on falling edge on  int0 
	
                       GICR |= (1 << INT0);  
        TCCR1A = 0;               // Clear TCCR1A
        TCCR1B |=  (1 << WGM12) | (1 << CS11);			// switch CTC Mode on, set prescaler to 8
   
					/*   8 000 000 Hz  /  8  /  OCR1A  =  times per second   */
   OCR1A   =   98;     // Set CTC compare value to ~10000 Hz at 8MHz AVR clock, with a prescaler of 256
   TIMSK |= (1 << TOIE1);  
   TIMSK |= (1 << OCIE1A); // Enable CTC interrupt
         
         TCNT1 = 0; 
	   
             #ifndef   OMNIDISPL
				TCCR2 |= (1 << CS21); //         enable timer2 with 8 prescaler
			 #else
	            TCCR2 |= (1 << CS22); 
			 #endif
                TIMSK |= (1 << TOIE2); //    // interrupt on  timer2 ovf

  
      sei();                    // Enable global interrupts
}

/**/
  //*****************************************************************************
  
         ISR(TIMER2_OVF_vect)
     { 
              static uint8_t flip;
			  int segs = 0;       
               #ifdef   TWODISPL
                 segs = TTLSEG / 2;    
		         if(!flip)  psvnSeg = &displays[0]; else psvnSeg = &displays[1]; 
               #else
                            psvnSeg = &displays[0]; segs = TTLSEG; 
               #endif

              if(led_r)
			  for(int i = 0; i < segs; i++) 
		       {
				  if(led_r)  {  *psvnSeg->setPort &= ~(1 << *(psvnSeg->setPin + i)); 
			                        }   else    {
		                           *psvnSeg->setPort |= (1 << *(psvnSeg->setPin + i));    }
			   }  

	     if(led_s) {   
			 			    #ifndef SEGSPLIT
							    **setSegmPorts = digits[psvnSeg->setVal[cDig]];
			                 #else
                       **setSegmPorts &=  ~(__PIN0  | __PIN1  | __PIN2  | __PIN3 | __PIN4 | __PIN5);
 	 	               **setSegmPorts |=   // port1
		             ((digits[psvnSeg->setVal[cDig]] &__PIN1)  | (digits[psvnSeg->setVal[cDig]] &__PIN2)
                    | (digits[psvnSeg->setVal[cDig]] &__PIN3) | (digits[psvnSeg->setVal[cDig]] &__PIN4)
						| (digits[psvnSeg->setVal[cDig]] &__PIN5) | (digits[psvnSeg->setVal[cDig]] &__PIN0));
                        	               
					 **(setSegmPorts + 1) &=   ~( __PIN3  | __PIN7);
                     **(setSegmPorts + 1) |=   // port2
				      /*  we have two pins (bits) with same number which is illegal within single byte,
					  to overcome this we simply shift the bit value to a deserved position  */
					  ((digits[psvnSeg->setVal[cDig]] &__PIN6)  >> 3) | (digits[psvnSeg->setVal[cDig]] &__PIN7);
							 #endif
				      }       else       {  
			 		      #ifndef SEGSPLIT
			         **setSegmPorts =~ digits[psvnSeg->setVal[cDig]]; 
		 			      #else
                      **setSegmPorts |=  __PIN0  | __PIN1  | __PIN2  | __PIN3 | __PIN4 | __PIN5;
 	 	              **setSegmPorts &= ~   // port1
					 ((digits[psvnSeg->setVal[cDig]] &__PIN1)  | (digits[psvnSeg->setVal[cDig]] &__PIN2)
                    | (digits[psvnSeg->setVal[cDig]] &__PIN3) | (digits[psvnSeg->setVal[cDig]] &__PIN4)
						| (digits[psvnSeg->setVal[cDig]] &__PIN5) | (digits[psvnSeg->setVal[cDig]] &__PIN0));
	        			 **(setSegmPorts + 1) |=   __PIN3  | __PIN7;
			             **(setSegmPorts + 1) &= ~   // port2
				     (((digits[psvnSeg->setVal[cDig]] &__PIN6)  >> 3) | (digits[psvnSeg->setVal[cDig]] &__PIN7));
						  #endif
		              }
                                     if(dotIsOn[flip] & (1 << cDig)) { if(led_s)  **(setSegmPorts + 1) |= (DOT);  
		                              else    **(setSegmPorts + 1) &=~ (DOT);  }

		           if(cDig < segs) {
	     if (led_r) { *psvnSeg->setPort |= (1 << psvnSeg->setPin[cDig]);  }  
	     else       { *psvnSeg->setPort &= ~(1 << psvnSeg->setPin[cDig]); } 
		                                  }
	                 if(++cDig > segs) { cDig = 0; flip ^= 1;    }
					
      }

 	

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
                ISR(TIMER1_COMPA_vect)  // 10 kHz / 0.1ms / 100ns
            {
       ++scaler;  counter += 1; 
		   if(firingAngle > 98)  {  PORT_TRIAC |= (1 << PIN_TRIAC); triacIsOn = 1; }  else {
			   //  delay to turn off triac
           if(triacIsOn == 1) {   PORT_TRIAC &= ~(1 << PIN_TRIAC);  triacIsOn = 0; }
	       if(halfwave == firingAngle) {  PORT_TRIAC |= (1 << PIN_TRIAC); triacIsOn = 1; }  
		   --halfwave;
		   }
            }

//*****************************************************************************
//                          Zero Crossing Detection
//*****************************************************************************
           ISR (INT0_vect)                                      
                   {
		 display = counter;
         counter = 0;
		 halfwave = 99;  // 
				   }
/**/



//*****************************************************************************
// rotary  encoder  opcodes

void rotaryEnc(void)
{
   static uint8_t encStat;   //  
   register int8_t encTmp = encPort >> 4; 
   register uint8_t tmp = (encTmp ^ (encStat & encMask)); 
   if(tmp)
       {
  // two bits should never change at once in Gray code
	                      {  
   encStat = (encStat << 2) | encTmp;
   // a single step encoder would change two bits per click
  
   // this is a two-step encoder: four bits are changed per each click 
          if(encStat == encPlus || (encStat & 0x0F) == (encPlus >> 4)) {  firingAngle++;  }    //display += 1;
          if(encStat == encMinus || (encStat & 0x0F) == (encMinus >> 4)) {  firingAngle--; } //display -= 1;
                          firingAngle = (firingAngle > 100) ? 0 : firingAngle;
                          firingAngle = (firingAngle < 0) ? 100 : firingAngle;                   
 	                     // stepper motor encoder exclusive mode
								       if(encStat == revPlus) {  encStat = encMinus >> 4; }  
								       if(encStat == revMinus) {  encStat = encPlus >> 4; }

          
			 } 
	   }
   }

//*****************************************************************************
         // convert binary to decimal
		  void btod(uint16_t valA, uint16_t valB)
          {  
	         register int8_t i;  uint16_t  x = 10, y = 1;

		   for( i = DISPVAL; i >= 0 ; i-- )
		          {
			  displays[0].setVal[i] =  valA % x / y; 
			         #ifdef   TWODISPL
			  displays[1].setVal[i] =  valB % x / y;
	                 #endif
			   x *= 10;  y *= 10;   
                  }
          }


		  void _demoA()
          { 
	                  static uint16_t cntA, cntB;
			 if(cntB++ > 250) {  counter += 1;   cntB = 0;  
			 btod(display, counter);  
			 dotIsOn[1] ^= 1; 
			}
            if(cntA++ > 500) { cntA = 0;    display += 1; 
			 dotIsOn[0] ^= 2;   
			}/**/  
		  }

	  void _demoB()
          { 
	                  static uint16_t cntA, cntB;
             static uint8_t x = 4, y = 1;
			 if(cntB++ > 100) {     cntB = 0;  //   
			 btod(firingAngle, display);  
			 dotIsOn[1] = 0;  dotIsOn[1] |= y;  y <<= 1;  if(y > 4) y = 1; // bitshift led blinker
			}
            if(cntA++ > 500) { cntA = 0;  //  
			
			 dotIsOn[0] = 0;  dotIsOn[0] ^= x;  x >>= 1;  if(x < 1) x = 4;  
			}/**/  
		  }
             int main(void)      
            { 

      init_io();
      timer_init();
             


//*****************************************************************************
//                                              main loop
//*****************************************************************************
   		      /*  */

		             
              for( ; ; )
	     {
                   
				 if(scaler > 10)  { scaler = 0;  rotaryEnc();   }
		  
		  if (TCNT0 >= 255) 
	                 {
		   _demoB();
     
								   }
				//*****************************************************************************
              

	     }    // end loop

	}            // end main
