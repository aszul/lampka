
/*
 This is an example of how simple driving a Neopixel can be
 This code is optimized for understandability and changability rather than raw speed
 More info at http://wp.josh.com/2014/05/11/ws2812-neopixels-made-easy/
*/

// Change this to be at least as long as your pixel string (too long will work fine, just be a little slower)

#define PIXELS 8  // Number of pixels in the string

// These values depend on which pin your string is connected to and what board you are using 
// More info on how to find these at http://www.arduino.cc/en/Reference/PortManipulation

// These values are for digital pin 8 on an Arduino Yun or digital pin 12 on a DueMilinove/UNO
// Note that you could also include the DigitalWriteFast header file to not need to to this lookup.

#define PIXEL_PORT  PORTB  // Port of the pin the pixels are connected to
#define PIXEL_DDR   DDRB   // Port of the pin the pixels are connected to
#define PIXEL_BIT   4      // Bit of the pin the pixels are connected to

// These are the timing constraints taken mostly from the WS2812 datasheets 
// These are chosen to be conservative and avoid problems rather than for maximum throughput 

#define T1H  900    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

#define RES 6000    // Width of the low gap between bits to cause a frame to latch

// Here are some convience defines for using nanoseconds specs to generate actual CPU delays

#define NS_PER_SEC (1000000000L)          // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (F_CPU)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )

// Actually send a bit to the string. We must to drop to asm to enusre that the complier does
// not reorder things and make it so the delay happens in the wrong place.

void sendBit( bool bitVal ) {
  
    if (  bitVal ) {        // 0 bit
      
    asm volatile (
      "sbi %[port], %[bit] \n\t"        // Set the output bit
      ".rept %[onCycles] \n\t"                                // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      "cbi %[port], %[bit] \n\t"                              // Clear the output bit
      ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      ::
      [port]    "I" (_SFR_IO_ADDR(PIXEL_PORT)),
      [bit]   "I" (PIXEL_BIT),
      [onCycles]  "I" (NS_TO_CYCLES(T1H) - 2),    // 1-bit width less overhead  for the actual bit setting, note that this delay could be longer and everything would still work
      [offCycles]   "I" (NS_TO_CYCLES(T1L) - 2)     // Minimum interbit delay. Note that we probably don't need this at all since the loop overhead will be enough, but here for correctness

    );
                                  
    } else {          // 1 bit

    // **************************************************************************
    // This line is really the only tight goldilocks timing in the whole program!
    // **************************************************************************


    asm volatile (
      "sbi %[port], %[bit] \n\t"        // Set the output bit
      ".rept %[onCycles] \n\t"        // Now timing actually matters. The 0-bit must be long enough to be detected but not too long or it will be a 1-bit
      "nop \n\t"                                              // Execute NOPs to delay exactly the specified number of cycles
      ".endr \n\t"
      "cbi %[port], %[bit] \n\t"                              // Clear the output bit
      ".rept %[offCycles] \n\t"                               // Execute NOPs to delay exactly the specified number of cycles
      "nop \n\t"
      ".endr \n\t"
      ::
      [port]    "I" (_SFR_IO_ADDR(PIXEL_PORT)),
      [bit]   "I" (PIXEL_BIT),
      [onCycles]  "I" (NS_TO_CYCLES(T0H) - 2),
      [offCycles] "I" (NS_TO_CYCLES(T0L) - 2)

    );
      
    }
    
    // Note that the inter-bit gap can be as long as you want as long as it doesn't exceed the 5us reset timeout (which is A long time)
    // Here I have been generous and not tried to squeeze the gap tight but instead erred on the side of lots of extra time.
    // This has thenice side effect of avoid glitches on very long strings becuase 

    
}  

  
void sendByte( uint8_t byte ) {
    
    for( uint8_t bit = 0 ; bit < 8 ; bit++ ) {
      
      sendBit( bitRead( byte , 7 ) );                // Neopixel wants bit in highest-to-lowest order
                                                     // so send highest bit (bit #7 in an 8-bit byte since they start at 0)
      byte <<= 1;                                    // and then shift left so bit 6 moves into 7, 5 moves into 6, etc
      
    }           
} 

/*

  The following three functions are the public API:
  
  ledSetup() - set up the pin that is connected to the string. Call once at the begining of the program.  
  sendPixel( r g , b ) - send a single pixel to the string. Call this once for each pixel in a frame.
  show() - show the recently sent pixel on the LEDs . Call once per frame. 
  
*/


// Set the specified pin up as digital out

inline void ledsetup() {
  
  bitSet( PIXEL_DDR , PIXEL_BIT );
  
}

void sendPixel( const uint8_t rgb[3] )  {  
  
  sendByte(rgb[0]);
  sendByte(rgb[1]);
  sendByte(rgb[2]);
  
}


// Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame

void show() {
  //_delay_us( (RES / 1000UL) + 1);       // Round up since the delay must be _at_least_ this long (too short might not work, too long not a problem)
 delayMicroseconds((RES / 1000UL) + 1);
}


/*

  That is the whole API. What follows are some demo functions rewriten from the AdaFruit strandtest code...
  
  https://github.com/adafruit/Adafruit_NeoPixel/blob/master/examples/strandtest/strandtest.ino
  
  Note that we always turn off interrupts while we are sending pixels becuase an interupt
  could happen just when we were in the middle of somehting time sensitive.
  
  If we wanted to minimize the time interrupts were off, we could instead 
  could get away with only turning off interrupts just for the very brief moment 
  when we are actually sending a 0 bit (~1us), as long as we were sure that the total time 
  taken by any interrupts + the time in our pixel generation code never exceeded the reset time (5us).
  
*/


/*
// Fill the dots one after the other with a color
// rewrite to lift the compare out of the loop
void colorWipe(uint8_t r , uint8_t g, uint8_t b, unsigned  char wait ) {
  for(unsigned int i=0; i<PIXELS; i+= (PIXELS/60) ) {
    
    cli();
    unsigned int p=0;
    
    while (p++<=i) {
        sendPixel(r,g,b);
    } 
     
    while (p++<=PIXELS) {
        sendPixel(0,0,0);  
      
    }
    
    sei();
    show();
    delay(wait);
  }
}
*/

/*
// Theatre-style crawling lights.
// Changes spacing to be dynmaic based on string size

#define THEATER_SPACING (PIXELS/20)

void theaterChase( uint8_t r , uint8_t g, uint8_t b, uint8_t wait ) {
  
  for (int j=0; j< 3 ; j++) {  
  
    for (int q=0; q < THEATER_SPACING ; q++) {
      
      unsigned int step=0;
      
      cli();
      
      for (int i=0; i < PIXELS ; i++) {
        
        if (step==q) {
          
          sendPixel( r , g , b );
          
        } else {
          
          sendPixel( 0 , 0 , 0 );
          
        }
        
        step++;
        
        if (step==THEATER_SPACING) step =0;
        
      }
      
      sei();
      
      show();
      delay(wait);
      
    }
    
  }
  
}
        
*/

/*

// I rewrite this one from scrtach to use high resolution for the color wheel to look nicer on a *much* bigger string
                                                                            
void rainbowCycle(uint8_t frames , unsigned int frameAdvance, unsigned int pixelAdvance ) {
  
  // Hue is a number between 0 and 3*256 than defines a mix of r->g->b where
  // hue of 0 = Full red
  // hue of 128 = 1/2 red and 1/2 green
  // hue of 256 = Full Green
  // hue of 384 = 1/2 green and 1/2 blue
  // ...
  
  unsigned int firstPixelHue = 0;     // Color for the first pixel in the string
  
  for(unsigned int j=0; j<frames; j++) {                                  
    
    unsigned int currentPixelHue = firstPixelHue;
       
    cli();    
        
    for(unsigned int i=0; i< PIXELS; i++) {
      
      if (currentPixelHue>=(3*256)) {                  // Normalize back down incase we incremented and overflowed
        currentPixelHue -= (3*256);
      }
            
      uint8_t phase = currentPixelHue >> 8;
      uint8_t step = currentPixelHue & 0xff;
                 
      switch (phase) {
        
        case 0: 
          sendPixel( ~step , step ,  0 );
          break;
          
        case 1: 
          sendPixel( 0 , ~step , step );
          break;

        case 2: 
          sendPixel(  step ,0 , ~step );
          break;
          
      }
      
      currentPixelHue+=pixelAdvance;                                      
      
                          
    } 
    
    sei();
    
    show();
    
    firstPixelHue += frameAdvance;
           
  }
}
*/


/*  
// I added this one just to demonstrate how quickly you can flash the string.
// Flashes get faster and faster until *boom* and fade to black.

void detonate( uint8_t r , uint8_t g , uint8_t b , unsigned int startdelayms) {
  while (startdelayms) {
    
    showColor( r , g , b );      // Flash the color 
    showColor( 0 , 0 , 0 );
    
    delay( startdelayms );      
    
    startdelayms =  ( startdelayms * 4 ) / 5 ;           // delay between flashes is halved each time until zero
    
  }

  // Then we fade to black....
  for( int fade=256; fade>0; fade-- ) {
    showColor( (r * fade) / 256 ,(g*fade) /256 , (b*fade)/256 );
  }
  showColor( 0 , 0 , 0 );
}
*/



////////////////////////////////////////////////////////////////////////////////////
#define K255 255
#define K171 171
#define K85  85

//#define SCALE8_C 1
#define SCALE8_AVRASM 1
#define LIB8_ATTINY
static inline uint8_t scale8( uint8_t i, uint8_t scale)
{
#if SCALE8_C == 1
    return
    ((int)i * (int)(scale) ) >> 8;
#elif SCALE8_AVRASM == 1
#if defined(LIB8_ATTINY)
    uint8_t work=0;
    uint8_t cnt=0x80;
    asm volatile(
        "LOOP_%=:                             \n\t"
        "  sbrc %[scale], 0             \n\t"
        "  add %[work], %[i]            \n\t"
        "  ror %[work]                  \n\t"
        "  lsr %[scale]                 \n\t"
        "  lsr %[cnt]                   \n\t"
        "brcc LOOP_%="
        : [work] "+r" (work), [cnt] "+r" (cnt)
        : [scale] "r" (scale), [i] "r" (i)
        :
      );
    return work;
#else
    asm volatile(
         /* Multiply 8-bit i * 8-bit scale, giving 16-bit r1,r0 */
         "mul %0, %1          \n\t"
         /* Move the high 8-bits of the product (r1) back to i */
         "mov %0, r1          \n\t"
         /* Restore r1 to "0"; it's expected to always be that */
         "clr __zero_reg__    \n\t"

         : "+a" (i)      /* writes to i */
         : "a"  (scale)  /* uses scale */
         : "r0", "r1"    /* clobbers r0, r1 */ );

    /* Return the result */
    return i;
#endif
#else
#error "No implementation for scale8 available."
#endif
}

static inline uint8_t scale8_video_LEAVING_R1_DIRTY( uint8_t i, uint8_t scale)
{
#if SCALE8_C == 1 || defined(LIB8_ATTINY)
    uint8_t j = (((int)i * (int)scale) >> 8) + ((i&&scale)?1:0);
    return j;
#elif SCALE8_AVRASM == 1
    uint8_t j=0;
    asm volatile(
        "  tst %[i]\n\t"
        "  breq L_%=\n\t"
        "  mul %[i], %[scale]\n\t"
        "  mov %[j], r1\n\t"
        "  breq L_%=\n\t"
        "  subi %[j], 0xFF\n\t"
        "L_%=: \n\t"
        : [j] "+a" (j)
        : [i] "a" (i), [scale] "a" (scale)
        : "r0", "r1");

    return j;
#else
#error "No implementation for scale8_video_LEAVING_R1_DIRTY available."
#endif
} 

void hsv2rgb_rainbow(uint8_t hue, uint8_t colors[3])
{
    // Yellow has a higher inherent brightness than
    // any other color; 'pure' yellow is perceived to
    // be 93% as bright as white.  In order to make
    // yellow appear the correct relative brightness,
    // it has to be rendered brighter than all other
    // colors.
    // Level Y1 is a moderate boost, the default.
    // Level Y2 is a strong boost.
    const uint8_t Y1 = 1;
    const uint8_t Y2 = 0;

    // G2: Whether to divide all greens by two.
    // Depends GREATLY on your particular LEDs
    const uint8_t G2 = 0;
    
    // Gscale: what to scale green down by.
    // Depends GREATLY on your particular LEDs
    const uint8_t Gscale = 0;

    uint8_t offset = hue & 0x1F; // 0..31
    
    // offset8 = offset * 8
    uint8_t offset8 = offset;
    {
        offset8 <<= 1;
        asm volatile("");
        offset8 <<= 1;
        asm volatile("");
        offset8 <<= 1;
    }
    
    uint8_t third = scale8( offset8, (256 / 3));
        
    memset(colors, 0, sizeof(colors));
    
    if( ! (hue & 0x80) ) {
        // 0XX
        if( ! (hue & 0x40) ) {
            // 00X
            //section 0-1
            if( ! (hue & 0x20) ) {
                // 000
                //case 0: // R -> O
                colors[0] = K255 - third;
                colors[1] = third;
            } else {
                // 001
                //case 1: // O -> Y
                if( Y1 ) {
                    colors[0] = K171;
                    colors[1] = K85 + third ;
                }
                if( Y2 ) {
                    colors[0] = K171 + third;
                    //uint8_t twothirds = (third << 1);
                    uint8_t twothirds = scale8( offset8, ((256 * 2) / 3));
                    colors[1] = K85 + twothirds;
                }
            }
        } else {
            //01X
            // section 2-3
            if( !  (hue & 0x20) ) {
                // 010
                //case 2: // Y -> G
                if( Y1 ) {
                    uint8_t twothirds = scale8( offset8, ((256 * 2) / 3));
                    colors[0] = K171 - twothirds;
                    colors[1] = K171 + third;
                }
                if( Y2 ) {
                    colors[0] = K255 - offset8;
                    colors[1] = K255;
                }
            } else {
                // 011
                // case 3: // G -> A
                colors[1] = K255 - third;
                colors[2] = third;
            }
        }
    } else {
        // section 4-7
        // 1XX
        if( ! (hue & 0x40) ) {
            // 10X
            if( ! ( hue & 0x20) ) {
                // 100
                //case 4: // A -> B
                //r = 0;
                uint8_t twothirds = scale8( offset8, ((256 * 2) / 3));
                colors[1] = K171 - twothirds;
                colors[2] = K85  + twothirds;
            } else {
                // 101
                //case 5: // B -> P
                colors[0] = third;
                colors[2] = K255 - third;
            }
        } else {
            if( !  (hue & 0x20)  ) {
                // 110
                //case 6: // P -- K
                colors[0] = K85 + third;
                colors[2] = K171 - third;
            } else {
                // 111
                //case 7: // K -> R
                colors[0] = K171 + third;
                colors[2] = K85 - third;
            }
        }
    }
    
    // This is one of the good places to scale the green down,
    // although the client can scale green down as well.
    if( G2 ) colors[1] = colors[1] >> 1;
    if( Gscale ) colors[1] = scale8_video_LEAVING_R1_DIRTY( colors[1], Gscale);
}

////////////////////////////////////////////////////////////////////////////////////


inline void interruptSetup() {
    const uint8_t directionButtonPin=3;
    const uint8_t constantButtonPin=0;
    // initialize the pushbutton pin as an input:
    // actually they are all inputs by default, so do nothing
    //DDRB |= _BV(buttonPin);
    
    //looks like it's 0 by default
    //MCUCR = 0; //The low level of INT0 generates an interrupt request.
    //MCUCR = 1; //Any logical change on INT0 generates an interrupt request.
    //MCUCR = 2; //The falling edge of INT0 generates an interrupt request.
    //MCUCR = 3; //The rising edge of INT0 generates an interrupt request.
    
    //GIMSK |= (1<<6); //Bit 6 INT0: External Interrupt Request 0 Enable
    GIMSK |= (1<<5); //Bit 5 PCIE: Pin Change Interrupt Enable
    PCMSK |= (1<<directionButtonPin); //PCINT[5:0]: Pin Change Enable Mask 5:0
    PCMSK |= (1<<constantButtonPin); //PCINT[5:0]: Pin Change Enable Mask 5:0
}

void setup() {
    interruptSetup();
    ledsetup();
}

const uint8_t delta = (256/PIXELS); //delta of hue between each pixel, spread out evenly
boolean constant=0;
int8_t direction = 1;
uint8_t constant_color_hue=0;
uint8_t led_colors[PIXELS][3];
uint8_t constant_color_index=0;
#define CONSTANT_COLORS_LEN 9
uint8_t current_hue=0;

const uint8_t test_pixel[3] = {32,32,32};

const uint8_t rgb_colors[CONSTANT_COLORS_LEN][3] = {
{255,0,0},
{255,127,0},
{255,255,0},
{0,255,0},
{0,255,255},
{0,0,255},
{255,0,255},
{255,127,255},
{255,255,255}
};

ISR(PCINT0_vect) {
    cli();
    if (PINB & (1<<3)) { //change direction
        constant = 0;
        direction += 2;
        direction = direction % 3;
        direction -= 1;
    }
    if (PINB & 1) { //constant color
        constant = 1;
        constant_color_hue += 32;
        constant_color_index++;
        constant_color_index %= CONSTANT_COLORS_LEN;
    }
    sei();
}

void clear_led_colors() {
    for (uint8_t index=0; index < PIXELS; index++) {
        led_colors[index][0]=0;
        led_colors[index][1]=0;
        led_colors[index][2]=0;
    }    
}

void set_one_led(uint8_t index) {
    led_colors[index][0]=255;
    led_colors[index][1]=255;
    led_colors[index][2]=255;
}

void show_all_led_colors() {
    cli();
    for (uint8_t index=0; index < PIXELS; index++) {
      sendPixel(led_colors[index]);
    }
    show();
    sei();
}

void loop() {
    clear_led_colors();
    for (uint8_t index=0; index < PIXELS; index++) {
        if (constant) {
            hsv2rgb_rainbow(constant_color_hue, led_colors[index]);
        } else {
            uint8_t offset = index * delta * direction;
            hsv2rgb_rainbow(current_hue + offset, led_colors[index]);
        }
    }
    current_hue++;
    show_all_led_colors();
    delay(20);
}
