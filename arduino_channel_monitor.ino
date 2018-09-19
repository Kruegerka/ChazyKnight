const byte INTERRUPT_PIN = 2;
const int ms_1.11 = ;

void setup() {
  //Setup the input capture pin to cause interrupts for the system.
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),input_capture_isr(),CHANGE);

  //Setup timer and start it
  
}

void loop() {
  //It should just sit here after initialization.
}

void input_capture_isr(){
  //put input_capture_isr code here
  
}

void timer_isr(){
  //put timer_isr code here
  
}
