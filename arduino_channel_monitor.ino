#include <avr/io.h>
#include <avr/interrupt.h>

//Function declaration for the input capture ISR
//void input_capture_isr();

//Timer register value for 1.11 ms.
//const int ms111 = 69; //For a prescaler of 256
const int ms111 = 17760; //For a prescaler of 1
const int test = 31250; //1s delay for prescaler of 256
const char startingMask = 0x80; //Starting mask of 1000000 for extracting the MSB to send first
volatile byte STATE; //0 for idle, 1 for busy, and 2 for collision
static byte PREV_STATE; //0 for idle, 1 for busy, and 2 for collision
static int lvl;

//Digital pin 2 is used for Input Capture interrupts from the bus.
const byte int_pin = 2;
//Digital pins 4 is used for the Green LED
const byte g_pin = 4;
//Digital pin 7 is used for the Yellow LED
const byte y_pin = 7;
//Digital pin 8 is for the red LED
const byte r_pin = 8;

//Use Timer 1 for the better resolution of a 16 bit timer for 65536 values.
//Timer syntax is x stands for timer number and y stands for register output number.

void inputCaptureISR(){
  STATE = 1;
  TCNT1 = 0; // Reset count value in the timer to 0.
  TIFR1 |= (1 << OCF1A);
}

void setup(){
  cli(); //Stop interrupts
  
  //Set Pin Directions
  pinMode(int_pin, INPUT); //Interrupt pin (Digital Pin 2)
  pinMode(g_pin, OUTPUT); //Green LED pin (Digital Pin 4)
  pinMode(y_pin, OUTPUT); //Yellow LED pin (Digital Pin 7)
  pinMode(r_pin, OUTPUT); //Red LED pin (Digital Pin 8)

  //Start with the IDLE state of gren LED on
  digitalWrite(g_pin,HIGH);
  digitalWrite(y_pin,LOW);
  digitalWrite(r_pin,LOW);
  STATE = 0; 
  PREV_STATE = 0;
  
  //Setup the input capture pin to cause interrupts for the system.
  attachInterrupt(digitalPinToInterrupt(int_pin),inputCaptureISR,CHANGE);

  //Setup timer and start it
  TCCR1A = 0; //Clear the A control register
  TCCR1B = 0; //Clear the B control register
  TCNT1 = 0; //Initialize counter start value to 0
  
  //OCR1A = test; //Test value for 500 ms timer.
  OCR1A = ms111; //Set the Output Compare Register to do 69 cycles before triggering interupt for 1.104 ms delay.
  TCCR1B |= (1 << WGM12); //Set to "Clear Timer on Compare" mode for autonomous restart on interrupt.
  TIMSK1 |= (1 << OCIE1A); //Set interrupt to trigger on a comparison match.
  //TCCR1B |= (1 << CS12); // set prescaler to 256 and start the timer
  TCCR1B |= (1 << CS10); // set prescaler to 1 and start the timer

  sei(); //Allow interrupts
}

void loop(){
  //It should just sit here after initialization.
  if(!(PREV_STATE == STATE)){
    switch(STATE){
      case 0:
         digitalWrite(g_pin,HIGH);
         digitalWrite(y_pin,LOW);
         digitalWrite(r_pin,LOW);
         PREV_STATE = 0;
         break;
      case 1:
         digitalWrite(g_pin,LOW);
         digitalWrite(y_pin,HIGH);
         digitalWrite(r_pin,LOW);
         PREV_STATE = 1;
         break;
      case 2:
         digitalWrite(g_pin,LOW);
         digitalWrite(y_pin,LOW);
         digitalWrite(r_pin,HIGH);
         PREV_STATE = 2;
         break;
    }
  }
}

ISR (TIMER1_COMPA_vect){
  /**
   * Change the state of the transmission line.
   * If the logic level is '1', then the transmission has ended, so it is idle.
   * If the logic level is '0', then a collision of "start" bits has occurred.
   */
   lvl = digitalRead(int_pin);
    if(lvl == HIGH){
      STATE = 0;
    } else if(lvl == LOW) {
      STATE = 2;
    }
}
