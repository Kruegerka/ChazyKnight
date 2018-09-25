#include <avr/io.h>
#include <avr/interrupt.h>

//Function declaration for the input capture ISR
//void input_capture_isr();

//Timer register value for 1.11 ms.
const int ms111 = 69;
const int test = 31250;
volatile enum {idle,busy,collision} STATE;

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
  //put input_capture_isr code here
  //Reset the timer
  TCNT1 = 0; // Reset count value in the timer to 0.
  
  //Change the state of the transmission line.
  //If the state is idle, then change it to busy because a packet is being transmitted.
  //If the state is busy, then remain busy because the transmission is not finished.
  //If the state is collision, then set to busy from restart of transmission.
  if(STATE == idle){
    STATE = busy;
    digitalWrite(g_pin,LOW); //Green LED on for IDLE
    digitalWrite(y_pin,HIGH); //Yellow LED off, not BUSY
  } else {
    STATE = busy;
    digitalWrite(g_pin,LOW); //Green LED off, not idle
    digitalWrite(y_pin,HIGH); //Yellow LED on for BUSY
    digitalWrite(r_pin,LOW); //Red LED off, not COLLISION
  }
}

void setup(){
  cli(); //Stop interrupts
  
  //Set Pin Directions
  pinMode(int_pin, INPUT); //Interrupt pin (Digital Pin 2)
  pinMode(g_pin, OUTPUT); //Green LED pin (Digital Pin 4)
  pinMode(y_pin, OUTPUT); //Yellow LED pin (Digital Pin 7)
  pinMode(r_pin, OUTPUT); //Red LED pin (Digital Pin 8)

  //Start with the IDLE state of gren LED on
  digitalWrite(g_pin,LOW);
  digitalWrite(y_pin,LOW);
  digitalWrite(r_pin,LOW);
  STATE = idle;
  
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
  TCCR1B |= (1 << CS12); // set prescaler to 256 and start the timer

  sei(); //Allow interrupts
}

void loop(){
  //It should just sit here after initialization.
}

ISR (TIMER1_COMPA_vect){
  //put timer_isr code here

  //Read the logic level of the bus (Digital Pin 2)
  byte v_lvl = digitalRead(int_pin);

  /**
   * Change the state of the transmission line.
   * If the logic level is '1', then the transmission has ended, so it is idle.
   * If the logic level is '0', then a collision of "start" bits has occurred.
   */
  if(v_lvl == '1'){
    STATE = idle;
    digitalWrite(g_pin,HIGH); //Green LED on for IDLE
    digitalWrite(y_pin,LOW); //Yellow LED off, not BUSY
    digitalWrite(r_pin,LOW); //Red LED off, not COLLISION
  } else {
    STATE = collision;
    digitalWrite(g_pin,LOW); //Green LED off, not IDLE
    digitalWrite(y_pin,LOW); //Yellow LED off, not BUSY
    digitalWrite(r_pin,HIGH); //Red LED on for COLLISION
  }
  /**
   * This is only here for Timer1 testing purposes.
   */
  /*if(STATE == idle){
    digitalWrite(g_pin,HIGH); //Green LED on for IDLE
    STATE = busy;
  } else {
    digitalWrite(g_pin,LOW); //Green LED on for IDLE
    STATE = idle;
  }*/
}
