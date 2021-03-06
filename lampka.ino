
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

void sendPixel(uint8_t rgb[3])  {
  
  /*for (uint8_t index=0; index < 3; index++) {
    sendByte(rgb[index]);
  }*/
  sendByte(rgb[0]);
  sendByte(rgb[1]);
  sendByte(rgb[2]);
}


// Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame

void show() {
 delayMicroseconds((RES / 1000UL) + 1);// Round up since the delay must be _at_least_ this long (too short might not work, too long not a problem)
}


////////////////////////////////////////////////////////////////////////////////////
#define K255 255
#define K171 171
#define K85  85

//#define SCALE8_C 1
#define SCALE8_AVRASM 1
#define LIB8_ATTINY
static uint8_t scale8( uint8_t i, uint8_t scale)
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
    const uint8_t Y1 = 0;
    const uint8_t Y2 = 1;

    uint8_t offset = hue & 0x1F; // 0..31
    
    // offset8 = offset * 8
    /*
    uint8_t offset8 = offset;
    {
        offset8 <<= 1;
        asm volatile("");
        offset8 <<= 1;
        asm volatile("");
        offset8 <<= 1;
    }*/
    uint8_t offset8 = offset << 3;
    
    //uint8_t third = scale8( offset8, (256 / 3));
    uint8_t third = offset8 / 3;
    uint8_t twothirds = (third << 1);
        
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
                    //uint8_t twothirds = scale8( offset8, ((256 * 2) / 3));
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
                    
                    //uint8_t twothirds = scale8( offset8, ((256 * 2) / 3));
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
                //uint8_t twothirds = (third << 1);
                //uint8_t twothirds = scale8( offset8, ((256 * 2) / 3));
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
}

////////////////////////////////////////////////////////////////////////////////////

const uint8_t directionButtonPin=3;
const uint8_t constantButtonPin=0;

inline void interruptSetup() {
    // initialize the pushbutton pin as an input:
    DDRB |= _BV(constantButtonPin);
    DDRB |= _BV(directionButtonPin);
    
    // enable pull-up resistor
    PORTB |= _BV(constantButtonPin);    
    PORTB |= _BV(directionButtonPin);    

    PCMSK |= (1<<directionButtonPin); //PCINT[5:0]: Pin Change Enable Mask 5:0
    PCMSK |= (1<<constantButtonPin); //PCINT[5:0]: Pin Change Enable Mask 5:0
    GIMSK |= (1<<PCIE); //Bit 5 PCIE: Pin Change Interrupt Enable
}


inline void timerSetup() {
    //Clock Select clk I/O /1024 (From prescaler)
    //TCCR0B |= _BV(CS02) | _BV(CS00);
    TCCR0B |= _BV(CS02);
    //CTC mode
    //TCCR0A |= _BV(WGM01);
    
    //Output Compare Register A
    //OCR0A = 0xFF;

    //OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
    //TIMSK0 |= _BV(OCIE0A);
    TIMSK0 |= _BV(TOIE0);
}
void init() {
}

const uint8_t eeprom_address = 0;
const uint8_t delta = (256/PIXELS); //delta of hue between each pixel, spread out evenly
volatile boolean constant=0;
volatile uint8_t direction = 0;
uint8_t constant_color_hue=0;
uint8_t led_colors[PIXELS][3];
uint8_t constant_color_index=0;
uint8_t current_hue=0;
uint8_t timer_counter=0;
uint8_t timer_counter2=0;

ISR(PCINT0_vect) {
    cli();
    timer_counter=0;
    timer_counter2=0;
    if ((PINB & _BV(directionButtonPin)) == 0) { //change direction
        constant = 0;
        direction++;
        direction = direction % 3;
        /*direction += 2;
        direction = direction % 3;
        direction -= 1;*/
        
    }
    if ((PINB & _BV(constantButtonPin)) == 0) { //constant color
        constant = 1;
        constant_color_index++;
        constant_color_index = constant_color_index % 9; //8 is white
    }
    sei();
}

inline void set_all_white() {
    memset(led_colors, 255, sizeof(led_colors));
}

inline void clear_led_colors() {
    memset(led_colors, 0, sizeof(led_colors));
}


void set_into_EEDR() {
    /* set EEDR register with values from direction and constant */
    EEDR = get_mode();
}

uint8_t get_mode() {
    /* convert direction and constant into an 8 bit flag byte
    bits are mapped as follows:
    CIIII_DD
    C = constant
    I = constant_color_index
    D = direction
    */
    uint8_t mode = direction & 3;
    mode |= constant_color_index << 3;
    mode |= constant << 7;
    return mode;
}

void get_from_EEDR() {
    /* get values from EEDR and put into direction, constant nd constant_color_index 
    bits are mapped as follows:
    CIIII_DD
    C = constant
    I = constant_color_index
    D = direction
    */
    direction = EEDR & 3;
    constant_color_index = (EEDR & 0b01111000) >> 3;
    constant = EEDR >> 7;
}

ISR(TIM0_OVF_vect) {
    cli();
    // 255 ~ 8.5s
    // 255*60 = 8 minutes
    // 255*120 = 16 minutes
    // 255*240 = 32 minutes
    // 255*255 = 35 minutes
    if (++timer_counter == 0) {
        timer_counter2++;
        if (timer_counter2 > 250) {
            timer_counter2 = 0;
            //EEPROM_read();
            uint8_t mode = get_mode();
            if (mode != EEDR) {
                set_into_EEDR();
                //set_all_white();
                //show_all_led_colors();
                //delay(300);
                
                EEPROM_write();
                //delay(1000);
            }
        }
    }
    sei();
}


void EEPROM_write()
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE));
    /* Set Programming mode */
    EECR = (0<<EEPM1)|(0>>EEPM0);
    /* Set up address and data registers */
    EEARL = eeprom_address;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}

void EEPROM_read()
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE));
    /* Set up address register */
    EEARL = eeprom_address;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);
    /* Return data from data register */
}

void show_all_led_colors() {
    cli();
    for (uint8_t index=0; index < PIXELS; index++) {
        sendPixel(led_colors[index]);
    }
    show();
    sei();
}

void setup() {
    cli();
    interruptSetup();
    ledsetup();
    timerSetup();
    EEPROM_read();
    if (EEDR != 255) {
        get_from_EEDR();
    }
    sei();
}

void loop() {
    clear_led_colors();
    //set_all_white();
    uint8_t hue_to_compute = 0;
    for (uint8_t index=0; index < PIXELS; index++) {
        if (constant) {
            if (constant_color_index == 8) { //white
                led_colors[index][0]=255;
                led_colors[index][1]=255;
                led_colors[index][2]=255;
            } else {
            //if (constant_color_index != 8) { //not white
                hue_to_compute = (256/8) * constant_color_index;
                hsv2rgb_rainbow(hue_to_compute, led_colors[index]);
            }
        } else {
            uint8_t offset = index * delta * (direction-1);
            hue_to_compute = current_hue + offset;
            hsv2rgb_rainbow(hue_to_compute, led_colors[index]);
        }
    }
    current_hue++;
    show_all_led_colors();
    delay(100);
}
