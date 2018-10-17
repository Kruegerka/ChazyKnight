#include <avr/io.h>
#include <avr/interrupt.h>

//Function declaration for the input capture ISR
//void input_capture_isr();

//Timer register value for 1.11 ms.
//const int ms111 = 69; //For a prescaler of 256
const int ms111 = 17760; //For a prescaler of 1 on Timer1
const int ms1 = 125; //For a prescaler of 128 on Timer2
const int test = 31250;  //1s delay for prescaler of 256 on Timer1
const int delayTime = 500;
const unsigned int startingMask = 0x80; //Starting mask of 10000000 for extracting the MSB to send first
volatile byte STATE;            //0 for idle, 1 for busy, and 2 for collision
static byte PREV_STATE;         //0 for idle, 1 for busy, and 2 for collision
static int lvl;
static bool last_lvl;
static bool t2_en;

//Digital pin 2 is used for Input Capture interrupts from the bus.
const byte cm_pin = 2;
const byte rxtx_pin = 12;
const byte rx_pin = 13;
//Digital pins 4 is used for the Green LED
const byte g_pin = 4;
//Digital pin 7 is used for the Yellow LED
const byte y_pin = 7;
//Digital pin 8 is for the red LED
const byte r_pin = 8;

static char buf[150];
static char recBuf[150];
static int numberOfChars;
static char incomingByte;
//Use Timer 1 for the better resolution of a 16 bit timer for 65536 values.
//Timer syntax is x stands for timer number and y stands for register output number.

void inputCaptureISR()
{
  STATE = 1;
  TCNT1 = 0; // Reset count value in the timer to 0.
  TIFR1 |= (1 << OCF1A);
}

void rcvInputCaptureISR(){
  if(!t2_en){
    t2_en = true;
    lvl = digitalRead(rx_pin);
    if(lvl == HIGH){
      //buf location becomes 1
    } else if(lvl == LOW){
      //buf location becomes 0
    }
  }
  
  int count = *TCNT1;
  //Check if the edge occurred in the middle of the bit period
  if(count > 0.5*ms111){
    //Realign the clock
    TCNT2 = 0;
    //Read level of 2nd half of bit period
    lvl = digitalRead(rx_pin);
    if(lvl == HIGH){
      //buf location becomes 1
    } else if(lvl == LOW){
      //buf location becomes 0
    }
    TIFR2 |= (1 << OCF2A);
  }
}

void sendChar(char character)
{
  //Send HIGH if bitwise & with mask is not 0; otherwise set level to LOW
  unsigned int mask = startingMask;
  unsigned int sendingChar = character;
  //pinMode(rxrxtx_pin, OUTPUT);

  //Serial.print(character);
  for (int i = 0; i < 8; i++)
  {
    //Serial.print((sendingChar & mask));
    //Serial.print(" ");
    if ((sendingChar & mask) != 0)
    { //The bit to transmit is a 1.
      
      //Check if the level has to be flipped for the first half of the bit period. Must be low for 1.
      if (last_lvl)
      {
        digitalWrite(rxtx_pin, LOW);
        //digitalWrite(rxrxtx_pin, HIGH);
        last_lvl = !last_lvl;
      }
      //Still delay even if level doesn't change and hold for a half-bit period.
      delayMicroseconds(delayTime);
      //If the level  is 0, which it should be from previous steps, then invert and send high.
      if (!last_lvl)
      {
        digitalWrite(rxtx_pin, HIGH);
        //digitalWrite(rxrxtx_pin, LOW);
        last_lvl = !last_lvl;
      }
      //Hold high level for second half-bit period.
      delayMicroseconds(delayTime);
    }
    else
    { //The bit to transmit is a 0.
      //Check if the level has to be flipped for the first half of the bit period. Must be high for 0.
      if (!last_lvl)
      {
        digitalWrite(rxtx_pin, HIGH);
        //digitalWrite(rxrxtx_pin, LOW);
        last_lvl = !last_lvl;
      }
      //Still delay even if the level doesn't change and hold for a half-bit period.
      delayMicroseconds(delayTime);
      //If the level  is 0, which it should be from previous steps, then invert and send high.
      if (last_lvl)
      {
        digitalWrite(rxtx_pin, LOW);
        //digitalWrite(rxrxtx_pin, HIGH);
        last_lvl = !last_lvl;
      }
      //Hold low level for second half-bit period.
      delayMicroseconds(delayTime);
    }
    mask = (mask >> 1); //Shift the mask to the right 1 to extract the next bit.
  }
  //pinMode(rxrxtx_pin, INPUT);
  //digitalWrite(rxrxtx_pin, HIGH);
}

void setup()
{
  cli(); //Stop interrupts

  //Set Pin Directions
  pinMode(cm_pin, INPUT); //Interrupt pin and rxtx pin (Digital Pin 2)
  pinMode(rxtx_pin, OUTPUT);
  pinMode(g_pin, OUTPUT);   //Green LED pin (Digital Pin 4)
  pinMode(y_pin, OUTPUT);   //Yellow LED pin (Digital Pin 7)
  pinMode(r_pin, OUTPUT);   //Red LED pin (Digital Pin 8)

  //Start with the IDLE state of gren LED on
  digitalWrite(g_pin, LOW);
  digitalWrite(y_pin, LOW);
  digitalWrite(r_pin, LOW);
  digitalWrite(rxtx_pin, HIGH);
  //digitalWrite(rxtx_pin, HIGH);
  STATE = 0;
  PREV_STATE = 0;
  

  //Setup the input capture pin to cause interrupts for the system.
  attachInterrupt(digitalPinToInterrupt(cm_pin), inputCaptureISR, CHANGE);

  //Setup timer 2 and start it
  TCCR1A = 0; //Clear the A control register
  TCCR1B = 0; //Clear the B control register
  TCNT1 = 0;  //Initialize counter start value to 0

  //OCR1A = test; //Test value for 500 ms timer.
  OCR1A = ms111;           //Set the Output Compare Register to do 69 cycles before triggering interupt for 1.104 ms delay.
  TCCR1B |= (1 << WGM12);  //Set to "Clear Timer on Compare" mode for autonomous restart on interrupt.
  TIMSK1 |= (1 << OCIE1A); //Set interrupt to trigger on a comparison match.
  //TCCR1B |= (1 << CS12); // set prescaler to 256 and start the timer
  TCCR1B |= (1 << CS10); // set prescaler to 1 and start the timer

  //Setup timer 2 and start it
  TCCR2A = 0; //Clear the A control register
  TCCR2B = 0; //Clear the B control register
  TCNT2 = 0;  //Initialize counter start value to 0

  //OCR1A = test; //Test value for 500 ms timer.
  OCR2A = ms111;           //Set the Output Compare Register to do 69 cycles before triggering interupt for 1.104 ms delay.
  TCCR2B |= (1 << WGM12);  //Set to "Clear Timer on Compare" mode for autonomous restart on interrupt.
  TIMSK2 |= (1 << OCIE2A); //Set interrupt to trigger on a comparison match.
  //TCCR2B |= (1 << CS20) | (1 << CS22); // set prescaler to 1 and start the timer
  t2_en = false;

  Serial.begin(9600);
  buf[150];
  numberOfChars = 0;

  sei(); //Allow interrupts
}

void loop()
{
  //It should just sit here after initialization.
  if (!(PREV_STATE == STATE))
  {
    switch (STATE)
    {
    case 0:
      digitalWrite(g_pin, HIGH);
      digitalWrite(y_pin, LOW);
      digitalWrite(r_pin, LOW);
      PREV_STATE = 0;
      break;
    case 1:
      digitalWrite(g_pin, LOW);
      digitalWrite(y_pin, HIGH);
      digitalWrite(r_pin, LOW);
      PREV_STATE = 1;
      break;
    case 2:
      digitalWrite(g_pin, LOW);
      digitalWrite(y_pin, LOW);
      digitalWrite(r_pin, HIGH);
      PREV_STATE = 2;
      break;
    }
  }
  // read the incoming byte:
  incomingByte = Serial.read();
  if (incomingByte == 13)//if user hits enter
  {
    Serial.print("\n"); //new line
    Serial.print("\r"); //return
    while(STATE != 0); //Wait until state becomes IDLE
    //while(STATE != 2){
    int i = 0;
    while((i < numberOfChars)&&(STATE !=2 ))
    {
      last_lvl = digitalRead(rxtx_pin);
      //Checks to see if indle state
      if (STATE != 2)//it got stuck in this loop as while loop
      {
        if (buf[i] != -1)
        {
          //Sends the char and then sets the buffer value to noise
          sendChar(buf[i]);
          //Serial.print(buf[i]);
          //buf[i] = -1;
        }
      }
      i++;
    }
    //}
    numberOfChars = 0;
  }
  else if (incomingByte > 0)
  {
    //Print char to Serial
    Serial.print((char)incomingByte);
    //store the char
    buf[numberOfChars] = incomingByte;
    numberOfChars++;
  }
  digitalWrite(rxtx_pin,HIGH); 
}

ISR(TIMER1_COMPA_vect)
{
  /**
   * Change the state of the transmission line.
   * If the logic level is '1', then the transmission has ended, so it is idle.
   * If the logic level is '0', then a collision of "start" bits has occurred.
   */
  lvl = digitalRead(cm_pin);
  if (lvl == HIGH)
  {
    STATE = 0;
  }
  else if (lvl == LOW)
  {
    STATE = 2;
  }
}

ISR(TIMER2_COMPA_vect){
  lvl = digitalRead(rx_pin);
  if(lvl == HIGH){
    //end transmission
  } else {
    //Transmission error
  }
}
