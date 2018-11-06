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
const int delayAdj = 25;
const unsigned int startingMask = 0x80; //Starting mask of 10000000 for extracting the MSB to send first
volatile byte STATE;            //0 for idle, 1 for busy, and 2 for collision
volatile byte PREV_STATE;         //0 for idle, 1 for busy, and 2 for collision
static int lvl;
//static int rec_lvl;
static bool last_lvl;
volatile static bool t2_en;

//Digital pin 2 is used for Input Capture interrupts from the bus.
const byte cm_pin = 2;
const byte rxtx_pin = 12;
const byte rx_pin = 3;
//Digital pins 4 is used for the Green LED
const byte g_pin = 4;
//Digital pin 7 is used for the Yellow LED
const byte y_pin = 7;
//Digital pin 8 is for the red LED
const byte r_pin = 8;

const byte SYNCHRONIZATION = 0x55;
const byte VERSION_NUMBER = 0x01;
const byte SOURCE_ADDR = 0x01;
const byte DEST_ADDR = 0x03; //Testing only
const byte CRC_OFF = 0x00;
const byte CRC_ON = 0x01;
const byte NO_CRC_STRING = 0xAA;
const byte MAX_MESSAGE_LENGTH = 255;
const short CRC_POLYNOMIAL = 0x0107; //0000 000|1 0000 0111| Where 100000111 is the CRC polynomial

static char buf[300];
volatile static char recBuf[300];
static int numberOfChars;
static char incomingByte;
volatile static int charCnt;
volatile static int recCnt;
volatile static char charRcvd;
static bool rstEdge;
static bool rec_lvl;
static bool reTrans = false;
static byte unsuccessCnt;
struct Packet {
	byte synch = SYNCHRONIZATION;
	byte vers = VERSION_NUMBER;
	byte source = SOURCE_ADDR;
	byte destination = DEST_ADDR;
	byte leng;
	byte CRC_flag = CRC_OFF;
	char message[MAX_MESSAGE_LENGTH];
	byte FCS;
};
volatile static struct Packet rxPacket;
volatile static struct Packet txPacket;
long randomBackoff;
//Use Timer 1 for the better resolution of a 16 bit timer for 65536 values.
//Timer syntax is x stands for timer number and y stands for register output number.

void inputCaptureISR()
{
  STATE = 1;
  TCNT1 = 0; // Reset count value in the timer to 0.
  TIFR1 |= (1 << OCF1A);
}

void rcvInputCaptureISR(){
  //Serial.print("Arrived at receive ISR.");
  //rec_lvl = digitalRead(rx_pin);
  rec_lvl = !rec_lvl;
  if(!t2_en){
    t2_en = true;
    //lvl = digitalRead(rx_pin);
    //if(rec_lvl == HIGH){
    if(rec_lvl == 1){
      //buf location becomes 1
      charRcvd = (charRcvd << 1) + 1;
      //Serial.println("F1");
    //} else if(rec_lvl == LOW){
    } else if(rec_lvl == 0){
      //buf location becomes 0
      charRcvd = (charRcvd << 1);
      //Serial.println("F0");
    }
    recCnt++;
  } else {
  
    int count = TCNT2;
    //Check if the edge occurred in the middle of the bit period
    if(count <= (0.75*ms1)){
      /*Serial.print("TCNT2 is ");
      Serial.println(count);*/
      if(!rstEdge){
        //if(recCnt>0){
          rstEdge = true;
        //}
      } else {
        //Read level of 2nd half of bit period
        //lvl = digitalRead(rx_pin);
        //if(rec_lvl == HIGH){
        if(rec_lvl == 1){
          //buf location becomes 1
          charRcvd = (charRcvd << 1) + 1;
          //Serial.println("E1");
        //} else if(rec_lvl == LOW){
        } else if(rec_lvl == 0){
          //buf location becomes 0
          charRcvd = (charRcvd << 1);
          //Serial.println("E0");
        }
        recCnt++;
        rstEdge = false;
      }
    } else {
      //Serial.println("Got a bit.");
      //Realign the clock
  
      //Read level of 2nd half of bit period
      //lvl = digitalRead(rx_pin);
      //if(rec_lvl == HIGH){
      if(rec_lvl == 1){
        //buf location becomes 1
        charRcvd = (charRcvd << 1) + 1;
        //Serial.println("B1");
      //} else if(rec_lvl == LOW){
      } else if(rec_lvl == 0){
        //buf location becomes 0
        charRcvd = (charRcvd << 1);
        //Serial.println("B0");
      }
      recCnt++;
    }
  }
  if(recCnt == 8){
    recBuf[charCnt] = charRcvd;
    //Serial.println(charRcvd);
    charCnt++;
    recCnt = 0;
    charRcvd = 0;
    //rstEdge = false;
    //Serial.println(rstEdge);
 }
 /*Serial.print("recCnt is ");
 Serial.println(recCnt);
 Serial.println(rstEdge);*/
 TCNT2 = 0;
 TIFR2 |= (1 << OCF2A);
}

void sendChar(char character)
{
  //Send HIGH if bitwise & with mask is not 0; otherwise set level to LOW
  unsigned int mask = startingMask;
  unsigned int sendingChar = character;
  //pinMode(rxrxtx_pin, OUTPUT);
  if(!reTrans){
    //Serial.print(character);
    for (int i = 0; i < 8; i++)
    {
      //Serial.print((sendingChar & mask));
      //Serial.print(" ");
      if(!reTrans){
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
          delayMicroseconds(delayTime-delayAdj);
          //If the level  is 0, which it should be from previous steps, then invert and send high.
          if (!last_lvl)
          {
            digitalWrite(rxtx_pin, HIGH);
            //digitalWrite(rxrxtx_pin, LOW);
            last_lvl = !last_lvl;
          }
          //Hold high level for second half-bit period.
          delayMicroseconds(delayTime-delayAdj);
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
          delayMicroseconds(delayTime-delayAdj);
          //If the level  is 0, which it should be from previous steps, then invert and send high.
          if (last_lvl)
          {
            digitalWrite(rxtx_pin, LOW);
            //digitalWrite(rxrxtx_pin, HIGH);
            last_lvl = !last_lvl;
          }
          //Hold low level for second half-bit period.
          delayMicroseconds(delayTime-delayAdj);
        }
        mask = (mask >> 1); //Shift the mask to the right 1 to extract the next bit.
      }
    }
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
  pinMode(rx_pin, INPUT);
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
  attachInterrupt(digitalPinToInterrupt(rx_pin), rcvInputCaptureISR, CHANGE);

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
  OCR2A = ms1;           //Set the Output Compare Register to do 69 cycles before triggering interupt for 1.104 ms delay.
  TCCR2B |= (1 << WGM12);  //Set to "Clear Timer on Compare" mode for autonomous restart on interrupt.
  TIMSK2 |= (1 << OCIE2A); //Set interrupt to trigger on a comparison match.
  TCCR2B |= (1 << CS20) | (1 << CS22); // set prescaler to 1 and start the timer
  t2_en = false;

  Serial.begin(9600);
  //buf[150];
  //recBuf[150];
  numberOfChars = 0;
  recCnt = 0;
  charCnt = 0;
  charRcvd = 0;
  rec_lvl = 1;
  rstEdge = false;

  randomSeed(analogRead(14));

  sei(); //Allow interrupts
}

long generateRandomBackOff(){
    //Nmax = 200
    //return = (N/nmax)*1s
    return random(200)*5.00; //miliseconds
}

//The following is what Kyle found
/*uint8_t crc8_ccitt(uint8_t inCrc, int inData)
{
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for (i = 0; i< 8; i++){
    if((data & 0x80) != 0){
      data <<= 1;
      data ^= 0x07;
    }
    else{
      data <<= 1;
    }
  }
  return data;
}*/

//The following is Chas' modification; to do the long string, repeat where inData becomes CRC result of the one before (cascading/recursive)
char crc8_ccitt(short inCrc, byte inData)
{
  uint8_t i;
  short data = inData;
  //Serial.println(inCrc,BIN);
  //Serial.println(data,BIN);

  for (i = 0; i < 8; i++){
    if((data & 0x100) == 0){
      data <<= 1;
      //Serial.println(data,BIN);
    }
    else{
      data ^= inCrc;
      i -= 1; //Only decrement for shifting
    }
    //Serial.println(data,BIN);
  }
  if((data & 0x100) != 0){
    data ^= inCrc;
  }
  //Serial.println(data,BIN);
  return (char)(data & 0xFF); //Ideally returns lower half of CRC.
}

void loop()
{
  //It should just sit here after initialization.
  if (PREV_STATE != STATE)
  {
    switch (STATE)
    {
    case 0:
      digitalWrite(g_pin, HIGH);
      digitalWrite(y_pin, LOW);
      digitalWrite(r_pin, LOW);
      break;
    case 1:
      digitalWrite(g_pin, LOW);
      digitalWrite(y_pin, HIGH);
      digitalWrite(r_pin, LOW);
      break;
    case 2:
      digitalWrite(g_pin, LOW);
      digitalWrite(y_pin, LOW);
      digitalWrite(r_pin, HIGH);
      break;
    }
    PREV_STATE = STATE;
  }
  // read the incoming byte:
  incomingByte = Serial.read();
  if (incomingByte == 13)//if user hits enter
  {
    Serial.print("\n\r"); //new line
    
    //Serial.print("\r"); //return
	  txPacket.leng = numberOfChars;
    while(STATE == 1){delay(generateRandomBackOff());}; //Wait until state becomes IDLE after random amount of time
    charCnt = 0;
    recCnt = 0;
    charRcvd = 0;
    //Calculate CRC
    bool successful = false;
    reTrans = true;
    unsuccessCnt = 0;
    while((!successful) && (unsuccessCnt < 20)){
      if(!successful){
        delay(generateRandomBackOff());
        reTrans = false;
        sendChar(txPacket.synch);
        sendChar(txPacket.vers);
        sendChar(txPacket.source);
        sendChar(txPacket.destination); //Eventually make this variable
        sendChar(txPacket.leng);
        sendChar(txPacket.CRC_flag);
        int i = 0;
        while((i < txPacket.leng) && (!reTrans)){
          sendChar(txPacket.message[i]);
          i++;
        }
        if(txPacket.CRC_flag == CRC_ON){
          byte fcs;
          byte cascadeByte;
          fcs = crc8_ccitt(CRC_POLYNOMIAL,txPacket.message[0]); //Calculate off first byte
          //Serial.print("fcs:"); Serial.println(fcs,BIN);
          for(int i = 1; i < numberOfChars; i++){
            cascadeByte = fcs ^ txPacket.message[i];
            //Serial.print("txPack:"); Serial.println(txPacket.message[i],BIN);
            //Serial.print("casc:"); Serial.println(cascadeByte,BIN);
            fcs = crc8_ccitt(CRC_POLYNOMIAL, cascadeByte); //Calulate with previous
           // Serial.print("cascfcs"); Serial.println(fcs,BIN); 
          }
          txPacket.FCS = fcs;
          sendChar(txPacket.FCS); //Calculate FCS first
        } else {
          txPacket.FCS = NO_CRC_STRING;
          sendChar(txPacket.FCS);
        }
        if(!reTrans){
          numberOfChars = 0;
          successful = true;
          unsuccessCnt = 0;
        }
      }
      unsuccessCnt++;
    }
  }
  
  else if (incomingByte > 0)
  {
    //Print char to Serial
    Serial.print((char)incomingByte);
    //store the char
    //buf[numberOfChars] = incomingByte;
	  txPacket.message[numberOfChars] = incomingByte;
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
    reTrans = true;
  }
}

ISR(TIMER2_COMPA_vect){
  lvl = digitalRead(rx_pin);
  //Serial.println("Timer 2 timed out.");
  
  if(lvl == HIGH){
    //end transmission
    if(t2_en && (STATE == 0)){
      if(charCnt != 0){
        //Serial.print("R: ");
        byte l = recBuf[4];

    		//Extract destination
    		rxPacket.destination = recBuf[3];
        if((rxPacket.destination == 1) || (rxPacket.destination == 2)){
          //Extract source
          rxPacket.source = recBuf[2];


          
      		//Extract length
      		rxPacket.leng = l;
      		//Extract CRC flag
      		rxPacket.CRC_flag = recBuf[5];
          //Extract the FCS for CRC checking
          byte fcs = 0;
          byte cascadeByte;
          if(rxPacket.CRC_flag == CRC_ON){
            fcs = crc8_ccitt(CRC_POLYNOMIAL,rxPacket.message[0]); //Calculate off first byte
            for(int i = 1; i < txPacket.leng; i++){
              cascadeByte = fcs ^ rxPacket.message[i];
              fcs = crc8_ccitt(CRC_POLYNOMIAL, cascadeByte); //Calulate with previous 
            }
            rxPacket.FCS = recBuf[6+l]; //The FCS is immediately after the message, if there is one.
            cascadeByte = fcs ^= rxPacket.FCS;
            fcs = crc8_ccitt(CRC_POLYNOMIAL, cascadeByte);
            //Serial.print("Transmitted FCS = "); Serial.println((txPacket.FCS));
            //Serial.print("Received Calculated FCS = "); Serial.println((fcs));
          } else {
            rxPacket.FCS = recBuf[6+1];
            //Serial.print("Transmitted FCS = "); Serial.println((txPacket.FCS));
          }
          if(fcs == 0){
            switch(rxPacket.source){
              case 3:
                Serial.print("From Adam: ");
                break;
              case 4:
                Serial.print("From Connor: ");
                break;
              case 5:
                Serial.print("From Michael: ");
                break;
              case 6:
                Serial.print("From Eli: ");
                break;
              case 7:
                Serial.print("From Guenther: ");
                break;
              case 8:
                Serial.print("From Nathan: ");
                break;
              case 9:
                Serial.print("From Jesse: ");
                break;
              case 10:
                Serial.print("From Robert: ");
                break;
              case 11:
                Serial.print("From Jeff: ");
                break;
              case 12:
                Serial.print("From Ashwin: ");
                break;
              case 13:
                Serial.print("From Saif: ");
                break;
              case 14:
                Serial.print("From Mitchell: ");
                break;
              case 15:
                Serial.print("From Max: ");
                break;
              case 16:
                Serial.print("From Chris: ");
                break;
              case 17:
                Serial.print("From John: ");
                break;
              case 254:
                Serial.print("From Dr. Rothe: ");
                break;
              case 255:
                Serial.print("From Dr. Meier: ");
                break;
              case 0:
                Serial.print("Broadcast: ");
                break;
              default:
                Serial.print("Uknown Address: ");
                break;
            }
        		//Extract message if there is one
        		if(rxPacket.leng > 0){
        			for(int i = 0; i < rxPacket.leng; i++){
        				rxPacket.message[i] = recBuf[6+i];
        				Serial.print(recBuf[6+i]);
        			}
        		}
          } else {
            Serial.print("Message received in error and was dropped."); Serial.print("FCS Result = "); Serial.println(fcs);
          }
          Serial.print("\n\r"); //new line
          /*Serial.print("SYNC = "); Serial.println((rxPacket.synch));
          Serial.print("VERSION = "); Serial.println((rxPacket.vers));
          Serial.print("SOURCE = "); Serial.println((rxPacket.source));
          Serial.print("DESTINATION = "); Serial.println((rxPacket.destination));
          Serial.print("LENGTH = "); Serial.println((rxPacket.leng));
          Serial.print("CRC FLAG = "); Serial.println((rxPacket.CRC_flag));
          Serial.print("Received FCS = "); Serial.println((rxPacket.FCS));*/
  
          recCnt = 0;
          charCnt =0;
          charRcvd = 0;
          t2_en = false;
        }
      } else {
        //Serial.println("-I-");
        charCnt = 0;
        recCnt = 0;
        charRcvd = 0;
        t2_en = false;
      }
      rstEdge = false;
      rec_lvl = 1;
    }
  } //else {
    ////Transmission error
  //}
}
