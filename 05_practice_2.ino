
#define PIN_LED 7
unsigned int count, toggle, flag;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 1;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop(){
  flag =1;
  while(1){
    blinkLED();
    flag=0;
  }
}

void blinkLED(void) {
  if(flag){
    toggle = toggle_state(toggle); //toggle LED value.
    digitalWrite(PIN_LED, toggle); // update LED status.
    delay(1000);
    for(int i=0;i<10;i++){
      toggle=toggle_state(toggle);
      digitalWrite(PIN_LED,toggle);
      delay(100);
    }
    toggle=toggle_state(toggle);
    digitalWrite(PIN_LED,toggle);
  }
}
int toggle_state(int toggle) {
  if (toggle ==0 )
    toggle++;
  else
    toggle--;
  return toggle;
}
